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
// YUYV format: 4 bytes per 2 pixels: Y0 U Y1 V
// Each pair of pixels shares U and V components
// row_stride: bytes per row (may include padding)
fn yuyv_to_rgb(yuyv_data: &[u8], width: u32, height: u32, row_stride: u32) -> Vec<u8> {
    let mut rgb_data = Vec::with_capacity((width * height * 3) as usize);
    let row_stride_bytes = row_stride as usize;
    
    for y in 0..height {
        let row_offset = (y as usize) * row_stride_bytes;
        
        for x in (0..width).step_by(2) {
            // Calculate byte index for this pixel pair
            // Each pair is 4 bytes: Y0 U Y1 V
            let byte_idx = row_offset + ((x / 2) * 4) as usize;
            
            if byte_idx + 3 >= yuyv_data.len() {
                eprintln!("Warning: Buffer overflow at row {}, col {}", y, x);
                break;
            }
            
            // Extract YUYV components: Y0 U Y1 V
            let y0 = yuyv_data[byte_idx] as i32;
            let u = yuyv_data[byte_idx + 1] as i32;
            let y1 = yuyv_data[byte_idx + 2] as i32;
            let v = yuyv_data[byte_idx + 3] as i32;
            
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
    use image::{ImageBuffer, ImageEncoder, Rgb, RgbImage};
    
    // Verify RGB data size
    let expected_size = (width * height * 3) as usize;
    if rgb_data.len() != expected_size {
        return Err(CameraError::Image(format!(
            "RGB data size mismatch: expected {} bytes, got {} bytes",
            expected_size, rgb_data.len()
        )));
    }
    
    // Create RGB image from raw data
    // ImageBuffer expects data in row-major order: [R, G, B, R, G, B, ...]
    let img: RgbImage = ImageBuffer::<Rgb<u8>, _>::from_raw(width, height, rgb_data.to_vec())
        .ok_or_else(|| CameraError::Image(format!(
            "Failed to create image buffer from {} bytes (expected {}x{}x3 = {} bytes)",
            rgb_data.len(), width, height, expected_size
        )))?;
    
    // Save first image for debugging
    static SAVED_FIRST: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if !SAVED_FIRST.swap(true, std::sync::atomic::Ordering::Relaxed) {
        if let Err(e) = img.save("debug_rgb_image.png") {
            eprintln!("Failed to save debug RGB image: {:?}", e);
        } else {
            println!("Saved debug RGB image to debug_rgb_image.png");
        }
    }
    
    // Encode to PNG using the newer API
    let mut png_bytes = Vec::new();
    {
        let encoder = image::codecs::png::PngEncoder::new(&mut png_bytes);
        encoder.write_image(
            img.as_raw(),
            width,
            height,
            image::ColorType::Rgb8.into(),
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
    
    // Check if format has stride (bytes per line might be different from width * 2)
    if let Some(stride) = actual_format.stride {
        println!("Stride (bytes per line): {} (expected: {})", 
            stride, 
            actual_format.width * 2);
        if stride != actual_format.width * 2 {
            eprintln!("Warning: Stride mismatch! This may cause image corruption.");
            eprintln!("  Using stride {} instead of expected {}", stride, actual_format.width * 2);
        }
    }

    // Create capture stream with 4 buffers
    let mut stream = Stream::with_buffers(&mut dev, v4l::buffer::Type::VideoCapture, 4)
        .map_err(|e| CameraError::Stream(format!("Failed to create stream: {:?}", e)))?;

    println!("Camera started, capturing frames...");
    println!("Publishing PNG images to /image topic (press Ctrl+C to stop)");

    // Main capture loop
    let mut frame_count = 0u64;
    // For 30 FPS, we need ~33ms between frames
    // But we'll capture as fast as possible and let the interval throttle
    let mut interval = tokio::time::interval(Duration::from_millis(33)); // ~30 FPS (33ms = 1000/30)
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    
    loop {
        interval.tick().await;
        
        // Dequeue a buffer (wait for frame)
        let (buffer, _meta) = stream.next()
            .map_err(|e| CameraError::Frame(format!("Failed to capture frame: {:?}", e)))?;

        frame_count += 1;

        // Get frame dimensions from format
        let width = actual_format.width;
        let height = actual_format.height;
        
        // Get stride (bytes per line) - use actual stride if available, otherwise calculate
        let row_stride = actual_format.stride.unwrap_or(width * 2);

        // Verify buffer size matches expected YUYV size
        let expected_size = (row_stride * height) as usize;
        if buffer.len() != expected_size {
            eprintln!("Warning: Buffer size mismatch. Expected {} bytes (stride {} * height {}), got {} bytes", 
                expected_size, row_stride, height, buffer.len());
            if buffer.len() < expected_size {
                eprintln!("Buffer is too small, skipping frame");
                continue;
            }
        }

        // Debug: Print first few bytes of first frame to verify format
        if frame_count == 1 {
            println!("First 16 bytes of raw frame: {:?}", 
                &buffer[..buffer.len().min(16)]);
            println!("Using stride: {} bytes per line", row_stride);
            if let Err(e) = std::fs::write("debug_raw_frame.bin", &buffer) {
                eprintln!("Failed to save raw frame: {:?}", e);
            } else {
                println!("Saved raw frame to debug_raw_frame.bin ({} bytes)", buffer.len());
            }
        }

        // Convert YUYV to RGB8 (using actual stride)
        let rgb_data = yuyv_to_rgb(&buffer, width, height, row_stride);
        
        // Verify RGB data size
        let expected_rgb_size = (width * height * 3) as usize;
        if rgb_data.len() != expected_rgb_size {
            eprintln!("Warning: RGB conversion size mismatch. Expected {} bytes, got {} bytes",
                expected_rgb_size, rgb_data.len());
        }
        
        // Save first RGB frame for debugging
        if frame_count == 1 {
            if let Err(e) = std::fs::write("debug_rgb_frame.raw", &rgb_data) {
                eprintln!("Failed to save RGB frame: {:?}", e);
            } else {
                println!("Saved RGB frame to debug_rgb_frame.raw ({} bytes)", rgb_data.len());
            }
        }
        
        // Convert RGB to PNG bytes
        let png_bytes = rgb_to_png(&rgb_data, width, height)?;
        let png_size = png_bytes.len(); // Save size before moving png_bytes
        
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
            step: png_size as u32, // For PNG, step is the total size
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
                png_size);
        }
    }
}
