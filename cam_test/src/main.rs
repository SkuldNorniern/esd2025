use ros2_client::{Context, Node, NodeOptions};
use v4l::prelude::*;
use v4l::video::Capture;
use v4l::Format;
use v4l::io::mmap::Stream;

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
                    if caps.device_caps().contains(v4l::device::CapabilityFlags::VIDEO_CAPTURE) {
                        return Ok(path.to_string());
                    }
                }
            }
        }
    }
    
    Err(CameraError::NoCameras)
}

fn main() -> Result<(), CameraError> {
    println!("Initializing camera node with V4L2...");

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| CameraError::Ros2(format!("Failed to create ROS2 context: {:?}", e)))?;
    let node = Node::new(&ctx, "camera_node", &NodeOptions::new().enable_rosout(true))
        .map_err(|e| CameraError::Ros2(format!("Failed to create ROS2 node: {:?}", e)))?;

    // Create publisher for /image topic
    println!("Creating ROS2 publisher on /image topic...");
    
    // TODO: Create actual publisher once ros2-client API is confirmed
    // let publisher = node.create_publisher::<sensor_msgs::msg::Image>(...)

    // Find and open V4L2 device
    let device_path = find_v4l_device()?;
    println!("Found camera device: {}", device_path);
    
    let mut dev = v4l::Device::with_path(&device_path)
        .map_err(|e| CameraError::DeviceOpen(format!("Failed to open device {}: {:?}", device_path, e)))?;

    // Query device capabilities
    let caps = dev.query_caps()
        .map_err(|e| CameraError::V4l(format!("Failed to query device capabilities: {:?}", e)))?;
    println!("Device: {}", caps.card());
    println!("Driver: {}", caps.driver());

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
        actual_format.width(), 
        actual_format.height(),
        actual_format.fourcc());

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
        let width = actual_format.width();
        let height = actual_format.height();
        let frame_size = buffer.len();

        // Convert frame data to ROS2 Image message format
        // For YUYV format:
        // - encoding: "yuyv" or we could convert to "rgb8"
        // - step: width * 2 (YUYV is 2 bytes per pixel)
        // - data: raw frame buffer
        
        // Create ROS2 Image message
        // sensor_msgs/Image structure:
        // - header (std_msgs/Header): stamp, frame_id
        // - height: u32
        // - width: u32
        // - encoding: String (e.g., "yuyv", "rgb8")
        // - is_bigendian: u8
        // - step: u32 (bytes per row)
        // - data: Vec<u8>
        
        // For YUYV format:
        // - encoding: "yuyv"
        // - step: width * 2
        // - data: frame buffer
        
        // TODO: Create actual ROS2 message using ros2-client API
        // The exact API depends on how ros2-client handles message types
        // Example structure (may need adjustment based on actual API):
        // let image_msg = sensor_msgs::msg::Image {
        //     header: ...,
        //     height,
        //     width,
        //     encoding: "yuyv".to_string(),
        //     is_bigendian: 0,
        //     step: width * 2,
        //     data: buffer.to_vec(),
        // };
        // publisher.publish(&image_msg)?;
        
        // Log frame capture info periodically to avoid flooding console
        if frame_count % 30 == 0 {
            println!("Captured frame #{}: {}x{} ({} bytes, sequence: {}) - ready to publish", 
                frame_count,
                width, height, 
                frame_size,
                meta.sequence);
        }
    }
}
