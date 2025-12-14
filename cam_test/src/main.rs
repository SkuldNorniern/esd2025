use ros_wrapper::{create_topic_sender, QosProfile, sensor_msgs::msg::Image};
use v4l::video::Capture;
use v4l::Format;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use std::time::Duration;

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
    Image(String),
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
            CameraError::Image(msg) => write!(f, "Image error: {}", msg),
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

// Convert RGB to PNG bytes
fn rgb_to_png(rgb_data: &[u8], width: u32, height: u32) -> Result<Vec<u8>, CameraError> {
    use image::{ImageBuffer, Rgb, RgbImage};
    
    // Create RGB image from raw data
    let img: RgbImage = ImageBuffer::<Rgb<u8>, _>::from_raw(width, height, rgb_data.to_vec())
        .ok_or_else(|| CameraError::Image("Failed to create image buffer".to_string()))?;
    
    // Encode to PNG
    let mut png_bytes = Vec::new();
    {
        let mut encoder = image::codecs::png::PngEncoder::new(&mut png_bytes);
        encoder.encode(
            &img,
            width,
            height,
            image::ColorType::Rgb8,
        )
        .map_err(|e| CameraError::Image(format!("Failed to encode PNG: {:?}", e)))?;
    }
    
    Ok(png_bytes)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing camera node with V4L2...");

    // Create topic sender using ros_wrapper
    println!("Creating ROS2 publisher on /image topic...");
    let (tx, _node) = create_topic_sender::<Image>(
        "camera_node",
        "/image",
        QosProfile::default(),
    )
    .map_err(|e| CameraError::Ros2(format!("Failed to create topic sender: {:?}", e)))?;

    println!("Publisher created successfully");
    println!("ROS_DOMAIN_ID: {}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    
    // Wait for publisher to establish connections
    println!("Waiting for publisher to establish connections with subscribers...");
    println!("  (DDS discovery can take 5-10 seconds for cross-device communication)");
    for _i in 1..=10 {
        tokio::time::sleep(Duration::from_secs(1)).await;
        print!(".");
        std::io::Write::flush(&mut std::io::stdout())?;
    }
    println!();
    println!("Publisher ready, starting to publish frames...");

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

    // Set format: 320x240 YUYV
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

    // Create capture stream with 4 buffers
    let mut stream = Stream::with_buffers(&mut dev, v4l::buffer::Type::VideoCapture, 4)
        .map_err(|e| CameraError::Stream(format!("Failed to create stream: {:?}", e)))?;

    println!("Camera started, capturing frames...");
    println!("Publishing PNG images to /image topic (press Ctrl+C to stop)");

    // Main capture loop
    let mut frame_count = 0u64;
    let mut interval = tokio::time::interval(Duration::from_millis(100)); // ~10 FPS
    
    loop {
        interval.tick().await;
        
        // Dequeue a buffer (wait for frame)
        let (buffer, _meta) = stream.next()
            .map_err(|e| CameraError::Frame(format!("Failed to capture frame: {:?}", e)))?;

        frame_count += 1;

        // Get frame dimensions from format
        let width = actual_format.width;
        let height = actual_format.height;

        // Convert YUYV to RGB8
        let rgb_data = yuyv_to_rgb(&buffer, width, height);
        
        // Convert RGB to PNG bytes
        let png_bytes = rgb_to_png(&rgb_data, width, height)?;
        
        // Create ROS2 sensor_msgs/Image message with PNG data
        // For PNG, we'll use encoding "png" and put PNG bytes in data field
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();
        
        let msg = Image {
            header: ros_wrapper::std_msgs::msg::Header {
                stamp: ros_wrapper::r2r::builtin_interfaces::msg::Time {
                    sec: now.as_secs() as i32,
                    nanosec: now.subsec_nanos(),
                },
                frame_id: "camera_frame".to_string(),
            },
            height,
            width,
            encoding: "png".to_string(),
            is_bigendian: 0,
            step: png_bytes.len() as u32, // For PNG, step is the total size
            data: png_bytes,
        };
        
        // Send message through channel
        if let Err(e) = tx.send(msg).await {
            eprintln!("Failed to send message: {:?}", e);
            // Continue anyway
        }
        
        // Log frame capture info periodically
        if frame_count % 30 == 0 {
            println!("Published frame #{}: {}x{} PNG ({} bytes)", 
                frame_count,
                width, height, 
                msg.data.len());
        }
    }
}
