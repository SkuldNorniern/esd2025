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

// Extract RGB data from buffer, handling stride/padding
// RGB3 format: 3 bytes per pixel (R, G, B)
// row_stride: bytes per row (may include padding)
fn extract_rgb(rgb_buffer: &[u8], width: u32, height: u32, row_stride: u32) -> Vec<u8> {
    let mut rgb_data = Vec::with_capacity((width * height * 3) as usize);
    let row_stride_bytes = row_stride as usize;
    let bytes_per_pixel = 3;
    
    for y in 0..height {
        let row_offset = (y as usize) * row_stride_bytes;
        
        for x in 0..width {
            let pixel_idx = row_offset + (x as usize * bytes_per_pixel);
            
            if pixel_idx + 2 >= rgb_buffer.len() {
                eprintln!("Warning: Buffer overflow at row {}, col {}", y, x);
                break;
            }
            
            // RGB3 format: R, G, B
            rgb_data.push(rgb_buffer[pixel_idx]);
            rgb_data.push(rgb_buffer[pixel_idx + 1]);
            rgb_data.push(rgb_buffer[pixel_idx + 2]);
        }
    }
    
    rgb_data
}

// Convert RGB to PNG bytes
fn rgb_to_png(rgb_data: &[u8], width: u32, height: u32) -> Result<Vec<u8>, CameraError> {
    use image::{ImageBuffer, Rgb, RgbImage};
    use std::io::Cursor;
    
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
    
    // Encode to PNG using write_to method
    let mut png_bytes = Vec::new();
    {
        let mut cursor = Cursor::new(&mut png_bytes);
        img.write_to(&mut cursor, image::ImageFormat::Png)
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

    // Set format: 320x240 RGB3 (24-bit RGB, no conversion needed)
    let width = 320;
    let height = 240;
    
    let format = Format::new(width, height, v4l::FourCC::new(b"RGB3"));
    dev.set_format(&format)
        .map_err(|e| CameraError::Format(format!("Failed to set format: {:?}", e)))?;

    // Verify the format was set correctly
    let actual_format = dev.format()
        .map_err(|e| CameraError::Format(format!("Failed to get format: {:?}", e)))?;
    println!("Camera format: {}x{} {:?}", 
        actual_format.width, 
        actual_format.height,
        actual_format.fourcc);
    
    // Check stride (bytes per line might be different from width * 3 for RGB3)
    let stride = actual_format.stride;
    println!("Stride (bytes per line): {} (expected: {})", 
        stride, 
        actual_format.width * 3);
    if stride != actual_format.width * 3 {
        eprintln!("Warning: Stride mismatch! This may cause image corruption.");
        eprintln!("  Using stride {} instead of expected {}", stride, actual_format.width * 3);
    }

    // Create capture stream with 4 buffers
    let mut stream = Stream::with_buffers(&mut dev, v4l::buffer::Type::VideoCapture, 4)
        .map_err(|e| CameraError::Stream(format!("Failed to create stream: {:?}", e)))?;

    // Start streaming (critical - without this, buffers won't be filled)
    stream.start()
        .map_err(|e| CameraError::Stream(format!("Failed to start streaming: {:?}", e)))?;

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
        let (buffer, meta) = stream.next()
            .map_err(|e| CameraError::Frame(format!("Failed to capture frame: {:?}", e)))?;

        frame_count += 1;

        // Get frame dimensions from format
        let width = actual_format.width;
        let height = actual_format.height;
        
        // Get stride (bytes per line) from format
        let row_stride = actual_format.stride;

        // Verify buffer size matches expected RGB3 size
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
            println!("Using stride: {} bytes per line (RGB3 format)", row_stride);
            if let Err(e) = std::fs::write("debug_raw_frame.bin", &buffer) {
                eprintln!("Failed to save raw frame: {:?}", e);
            } else {
                println!("Saved raw frame to debug_raw_frame.bin ({} bytes)", buffer.len());
            }
        }

        // Extract RGB data from buffer (handling stride/padding if needed)
        let rgb_data = if row_stride == width * 3 {
            // No padding, use buffer directly
            buffer.to_vec()
        } else {
            // Has padding, extract pixel data
            extract_rgb(&buffer, width, height, row_stride)
        };
        
        // Verify RGB data size
        let expected_rgb_size = (width * height * 3) as usize;
        if rgb_data.len() != expected_rgb_size {
            eprintln!("Warning: RGB data size mismatch. Expected {} bytes, got {} bytes",
                expected_rgb_size, rgb_data.len());
            if rgb_data.len() < expected_rgb_size {
                eprintln!("RGB data too small, skipping frame");
                continue;
            }
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
        
        // Re-queue the buffer so it can be filled with the next frame
        stream.queue(meta.index)
            .map_err(|e| CameraError::Frame(format!("Failed to re-queue buffer: {:?}", e)))?;
    }
}
