use ros2_client::{Context, NodeName, NodeOptions, MessageTypeName, Name, Message};
use ros2_client::rustdds::QosPolicies;
use v4l::video::Capture;
use v4l::Format;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use base64::{Engine as _, engine::general_purpose};

// Error type for camera operations
#[derive(Debug)]
enum CameraError {
    V4l(String),
    NoCameras,
    DeviceOpen(String),
    Format(String),
    Stream(String),
    Frame(String),
    Ros2(String),
    Message(String),
}

impl std::fmt::Display for CameraError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CameraError::V4l(msg) => write!(f, "V4L error: {}", msg),
            CameraError::NoCameras => write!(f, "No cameras found"),
            CameraError::DeviceOpen(msg) => write!(f, "Device open error: {}", msg),
            CameraError::Format(msg) => write!(f, "Format error: {}", msg),
            CameraError::Stream(msg) => write!(f, "Stream error: {}", msg),
            CameraError::Frame(msg) => write!(f, "Frame error: {}", msg),
            CameraError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            CameraError::Message(msg) => write!(f, "Message error: {}", msg),
        }
    }
}

impl std::error::Error for CameraError {}

fn find_v4l_device() -> Result<String, CameraError> {
    // Try common V4L2 device paths
    let device_paths = ["/dev/video0", "/dev/video1", "/dev/video2"];
    
    for path in device_paths.iter() {
        if std::path::Path::new(path).exists() {
            // Try to open it to verify it's a capture device
            if let Ok(dev) = v4l::Device::with_path(path) {
                if let Ok(caps) = dev.query_caps() {
                    // Check if device supports video capture
                    // Flags::VIDEO_CAPTURE is a bitflag from capability module
                    if caps.capabilities.contains(v4l::capability::Flags::VIDEO_CAPTURE) {
                        return Ok(path.to_string());
                    }
                }
            }
        }
    }
    
    Err(CameraError::NoCameras)
}

// Convert YUYV format to RGB8
// YUYV is packed format: Y0 U0 Y1 V0 Y2 U1 Y3 V1 ...
// Each pair of pixels shares U and V components
fn yuyv_to_rgb(yuyv_data: &[u8], width: u32, height: u32) -> Vec<u8> {
    let mut rgb_data = Vec::with_capacity((width * height * 3) as usize);
    
    for y in 0..height {
        for x in (0..width).step_by(2) {
            let idx = ((y * width + x) * 2) as usize;
            if idx + 3 >= yuyv_data.len() {
                break;
            }
            
            let y0 = yuyv_data[idx] as i32;
            let u = yuyv_data[idx + 1] as i32;
            let y1 = yuyv_data[idx + 2] as i32;
            let v = yuyv_data[idx + 3] as i32;
            
            // Convert YUV to RGB for first pixel
            let (r0, g0, b0) = yuv_to_rgb(y0, u, v);
            rgb_data.push(r0);
            rgb_data.push(g0);
            rgb_data.push(b0);
            
            // Convert YUV to RGB for second pixel (if not at end of row)
            if x + 1 < width {
                let (r1, g1, b1) = yuv_to_rgb(y1, u, v);
                rgb_data.push(r1);
                rgb_data.push(g1);
                rgb_data.push(b1);
            }
        }
    }
    
    rgb_data
}

// Convert YUV to RGB using standard conversion formula
fn yuv_to_rgb(y: i32, u: i32, v: i32) -> (u8, u8, u8) {
    let c = y - 16;
    let d = u - 128;
    let e = v - 128;
    
    let r = (298 * c + 409 * e + 128) >> 8;
    let g = (298 * c - 100 * d - 208 * e + 128) >> 8;
    let b = (298 * c + 516 * d + 128) >> 8;
    
    (
        r.max(0).min(255) as u8,
        g.max(0).min(255) as u8,
        b.max(0).min(255) as u8,
    )
}

fn main() -> Result<(), CameraError> {
    println!("Initializing camera node with V4L2...");

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| CameraError::Ros2(format!("Failed to create ROS2 context: {:?}", e)))?;
    
    // Create node using Context::new_node
    let node_name = NodeName::new("/", "camera_node")
        .map_err(|e| CameraError::Ros2(format!("Failed to create node name: {:?}", e)))?;
    let mut node = ctx
        .new_node(node_name, NodeOptions::new().enable_rosout(true))
        .map_err(|e| CameraError::Ros2(format!("Failed to create ROS2 node: {:?}", e)))?;

    // Create publisher for /image topic using std_msgs/String for base64-encoded images
    println!("Creating ROS2 publisher on /image topic...");
    let topic_name = Name::new("/", "image")
        .map_err(|e| CameraError::Ros2(format!("Failed to create topic name: {:?}", e)))?;
    let message_type = MessageTypeName::new("std_msgs", "String");
    
    // Create topic first, then use it to create publisher
    let image_topic = node
        .create_topic(&topic_name, message_type, &QosPolicies::default())
        .map_err(|e| CameraError::Ros2(format!("Failed to create topic: {:?}", e)))?;
    
    let mut publisher = node
        .create_publisher(&image_topic, None)
        .map_err(|e| CameraError::Ros2(format!("Failed to create publisher: {:?}", e)))?;
    
    println!("Publisher created successfully");

    // Find and open V4L2 device
    let device_path = find_v4l_device()?;
    println!("Found camera device: {}", device_path);
    
    let mut dev = v4l::Device::with_path(&device_path)
        .map_err(|e| CameraError::DeviceOpen(format!("Failed to open device {}: {:?}", device_path, e)))?;

    // Query device capabilities
    let caps = dev.query_caps()
        .map_err(|e| CameraError::V4l(format!("Failed to query device capabilities: {:?}", e)))?;
    println!("Device: {}", caps.card);
    println!("Driver: {}", caps.driver);

    // Set format: 320x240 YUYV (common V4L2 format, similar to YUV420)
    // YUYV is a packed YUV format that's widely supported
    let width = 320;
    let height = 240;
    
    let format = Format::new(width, height, v4l::FourCC::new(b"YUYV"));
    dev.set_format(&format)
        .map_err(|e| CameraError::Format(format!("Failed to set format: {:?}", e)))?;

    // Verify the format was set correctly
    let actual_format = dev.format()
        .map_err(|e| CameraError::Format(format!("Failed to get format: {:?}", e)))?;
    println!("Camera format: {}x{} {:?}", 
        actual_format.width, 
        actual_format.height,
        actual_format.fourcc);

    // Create capture stream with 4 buffers for double buffering
    let mut stream = Stream::with_buffers(&mut dev, v4l::buffer::Type::VideoCapture, 4)
        .map_err(|e| CameraError::Stream(format!("Failed to create stream: {:?}", e)))?;

    println!("Camera started, capturing frames...");
    println!("Publishing to /image topic (press Ctrl+C to stop)");

    // Main capture loop
    let mut frame_count = 0u64;
    loop {
        // Dequeue a buffer (wait for frame)
        // The stream.next() returns a Result with (buffer, metadata)
        let (buffer, meta) = stream.next()
            .map_err(|e| CameraError::Frame(format!("Failed to capture frame: {:?}", e)))?;

        frame_count += 1;

        // Get frame dimensions from format
        let width = actual_format.width;
        let height = actual_format.height;
        let frame_size = buffer.len();

        // Convert YUYV to RGB8 for ball_detect
        let rgb_data = yuyv_to_rgb(&buffer, width, height);
        
        // Base64 encode the RGB image data
        let base64_data = general_purpose::STANDARD.encode(&rgb_data);
        
        // Create JSON message with metadata and base64-encoded image
        // Format: {"width": 320, "height": 240, "encoding": "rgb8", "data": "base64_string"}
        let json_msg = format!(
            r#"{{"width":{},"height":{},"encoding":"rgb8","data":"{}"}}"#,
            width, height, base64_data
        );
        
        // Create ROS2 std_msgs/String message using the topic
        let mut msg = image_topic.create_message()
            .map_err(|e| CameraError::Message(format!("Failed to create message: {:?}", e)))?;
        
        // Set the "data" field of std_msgs/String
        msg.set("data", json_msg)
            .map_err(|e| CameraError::Message(format!("Failed to set message data: {:?}", e)))?;
        
        // Publish the message
        publisher.publish(msg)
            .map_err(|e| CameraError::Message(format!("Failed to publish message: {:?}", e)))?;
        
        // Log frame capture info periodically to avoid flooding console
        if frame_count % 30 == 0 {
            println!("Published frame #{}: {}x{} RGB ({} bytes raw, {} bytes base64, sequence: {})", 
                frame_count,
                width, height, 
                rgb_data.len(),
                base64_data.len(),
                meta.sequence);
        }
    }
}
