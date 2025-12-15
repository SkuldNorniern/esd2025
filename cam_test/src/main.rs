use ros_wrapper::{create_topic_sender, QosProfile, sensor_msgs::msg::Image};
use v4l::video::Capture;
use v4l::Format;
use v4l::io::mmap::Stream;
use v4l::io::traits::{CaptureStream, Stream as StreamTrait};
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
        }
    }
}

impl std::error::Error for CameraError {}

fn find_v4l_device() -> Result<String, CameraError> {
    // Use /dev/video2 specifically as requested
    let device_path = "/dev/video1";
    
    if std::path::Path::new(device_path).exists() {
        // Try to open it to verify it's a capture device
        if let Ok(dev) = v4l::Device::with_path(device_path) {
            if let Ok(caps) = dev.query_caps() {
                // Check if device supports video capture
                if caps.capabilities.contains(v4l::capability::Flags::VIDEO_CAPTURE) {
                    return Ok(device_path.to_string());
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

    // Set format: 640x640 MJPEG (Motion JPEG, compressed format)
    // MJPEG is more commonly supported by webcams and uses less bandwidth
    let width = 640;
    let height = 640;
    
    let format = Format::new(width, height, v4l::FourCC::new(b"MJPG"));
    dev.set_format(&format)
        .map_err(|e| CameraError::Format(format!("Failed to set format: {:?}", e)))?;

    // Verify the format was set correctly
    let actual_format = dev.format()
        .map_err(|e| CameraError::Format(format!("Failed to get format: {:?}", e)))?;
    println!("Camera format: {}x{} {:?}", 
        actual_format.width, 
        actual_format.height,
        actual_format.fourcc);
    
    // Check format - MJPEG is compressed, so stride is 0
    let stride = actual_format.stride;
    let is_mjpeg = actual_format.fourcc.to_string() == "MJPG";
    
    if is_mjpeg {
        println!("Format: MJPEG (compressed JPEG, stride is 0 for compressed formats)");
    } else {
        println!("Stride (bytes per line): {} (expected: {})", 
            stride, 
            actual_format.width * 3);
        if stride != actual_format.width * 3 && stride != 0 {
            eprintln!("Warning: Stride mismatch! This may cause image corruption.");
            eprintln!("  Using stride {} instead of expected {}", stride, actual_format.width * 3);
        }
    }
    
    // Store is_mjpeg for use in the capture loop
    let format_is_mjpeg = is_mjpeg;

    // Create capture stream with 4 buffers
    let mut stream = Stream::with_buffers(&mut dev, v4l::buffer::Type::VideoCapture, 4)
        .map_err(|e| CameraError::Stream(format!("Failed to create stream: {:?}", e)))?;

    // Start streaming (critical - without this, buffers won't be filled)
    stream.start()
        .map_err(|e| CameraError::Stream(format!("Failed to start streaming: {:?}", e)))?;

    println!("Camera started, waiting for stream to stabilize...");
    // Wait longer for the camera to start filling buffers (first frames are often empty/black)
    // Some cameras need 1-2 seconds to initialize
    for i in 1..=20 {
        tokio::time::sleep(Duration::from_millis(100)).await;
        if i % 5 == 0 {
            print!(".");
            std::io::Write::flush(&mut std::io::stdout())?;
        }
    }
    println!();
    if format_is_mjpeg {
        println!("Publishing MJPEG (compressed JPEG) images to /image topic (press Ctrl+C to stop)");
    } else {
        println!("Publishing RGB3/rgb24 (24-bit RGB 8-8-8) raw images to /image topic (press Ctrl+C to stop)");
    }

    // Main capture loop
    // Note: stream.next() is a blocking call, but for camera frames it should return quickly (~33ms)
    // If it blocks longer, there may be an issue with the camera
    let mut frame_count = 0u64;
    // For 30 FPS, we need ~33ms between frames
    // But we'll capture as fast as possible and let the interval throttle
    let mut interval = tokio::time::interval(Duration::from_millis(33)); // ~30 FPS (33ms = 1000/30)
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    
    loop {
        interval.tick().await;
        
        // Dequeue a buffer (wait for frame)
        // stream.next() is blocking - use block_in_place to avoid stalling async runtime
        // This allows the blocking call without moving the stream
        let frame_start = std::time::Instant::now();
        let result = tokio::task::block_in_place(|| {
            stream.next()
        });
        
        let elapsed = frame_start.elapsed();
        if elapsed > Duration::from_millis(100) {
            eprintln!("WARNING: stream.next() took {:?} (unusually long)", elapsed);
        }
        
        let (buffer, meta) = match result {
            Ok((buf, m)) => (buf, m),
            Err(e) => {
                eprintln!("ERROR: Failed to capture frame: {:?}", e);
                eprintln!("  Camera may have disconnected or stopped streaming.");
                eprintln!("  Frame count before error: {}", frame_count);
                // Don't exit immediately - try to continue in case it's a temporary error
                tokio::time::sleep(Duration::from_millis(100)).await;
                continue;
            }
        };

        frame_count += 1;
        
        // Log every frame initially, then periodically to verify we're capturing
        if frame_count <= 5 || frame_count % 30 == 0 {
            println!("Captured frame #{} (seq: {}, bytes: {})", 
                frame_count, meta.sequence, meta.bytesused);
        }

        // Get frame dimensions from format
        let width = actual_format.width;
        let height = actual_format.height;
        
        // Get stride (bytes per line) from format
        let row_stride = actual_format.stride;
        
        // For MJPEG, use the actual bytes used (compressed size)
        // For uncompressed formats, use stride * height
        let image_data = if format_is_mjpeg {
            // MJPEG: use only the bytes actually used (compressed JPEG data)
            // meta.bytesused contains the actual compressed size
            let bytes_used = meta.bytesused as usize;
            if bytes_used > buffer.len() {
                eprintln!("Warning: Bytes used ({}) exceeds buffer size ({}), using buffer size", 
                    bytes_used, buffer.len());
                &buffer[..]
            } else {
                &buffer[..bytes_used]
            }
        } else {
            // Uncompressed format: verify buffer size
            let expected_size = if row_stride > 0 {
                (row_stride * height) as usize
            } else {
                (width * height * 3) as usize // RGB3 fallback
            };
            
            if buffer.len() != expected_size {
                eprintln!("Warning: Buffer size mismatch. Expected {} bytes (stride {} * height {}), got {} bytes", 
                    expected_size, row_stride, height, buffer.len());
                if buffer.len() < expected_size {
                    eprintln!("Buffer is too small, skipping frame");
                    continue;
                }
            }
            &buffer[..]
        };

        // For MJPEG, check if it's a valid JPEG (starts with FF D8)
        // For uncompressed, check if buffer is all zeros
        let is_valid = if format_is_mjpeg {
            // MJPEG: check JPEG header (FF D8)
            image_data.len() >= 2 && image_data[0] == 0xFF && image_data[1] == 0xD8
        } else {
            // Uncompressed: check if not all zeros
            let sample_size = (image_data.len() / 10).min(1024);
            !(image_data.iter().take(sample_size).all(|&b| b == 0) &&
              image_data.iter().skip(image_data.len() / 2).take(sample_size).all(|&b| b == 0) &&
              image_data.iter().skip(image_data.len().saturating_sub(sample_size)).all(|&b| b == 0))
        };
        
        if !is_valid {
            if frame_count <= 10 {
                if format_is_mjpeg {
                    println!("Frame #{}: Invalid JPEG (seq: {}, bytesused: {}), skipping (camera may still be initializing)", 
                        frame_count, meta.sequence, meta.bytesused);
                } else {
                    println!("Frame #{}: Buffer is all zeros (seq: {}, bytesused: {}), skipping (camera may still be initializing)", 
                        frame_count, meta.sequence, meta.bytesused);
                }
                continue;
            } else if frame_count == 11 {
                eprintln!("ERROR: Camera is returning invalid frames!");
                eprintln!("  Sequence: {}, Bytes used: {}, Buffer size: {}", 
                    meta.sequence, meta.bytesused, buffer.len());
                eprintln!("  Continuing to publish (will stop warnings)...");
            }
        }
        
        // Debug: Print first few bytes of first valid frame to verify format
        if frame_count == 1 || (is_valid && frame_count <= 5) {
            println!("Frame #{}: First 16 bytes: {:?}", frame_count,
                &image_data[..image_data.len().min(16)]);
            println!("  Sequence: {}, Bytes used: {}, Buffer size: {}", 
                meta.sequence, meta.bytesused, buffer.len());
            if frame_count == 1 && is_valid {
                if format_is_mjpeg {
                    println!("Format: MJPEG (compressed JPEG)");
                    if let Err(e) = std::fs::write("debug_mjpeg_frame.jpg", image_data) {
                        eprintln!("Failed to save MJPEG frame: {:?}", e);
                    } else {
                        println!("Saved MJPEG frame to debug_mjpeg_frame.jpg ({} bytes)", image_data.len());
                    }
                } else {
                    println!("Using stride: {} bytes per line", row_stride);
                    if let Err(e) = std::fs::write("debug_raw_frame.bin", image_data) {
                        eprintln!("Failed to save raw frame: {:?}", e);
                    } else {
                        println!("Saved raw frame to debug_raw_frame.bin ({} bytes)", image_data.len());
                    }
                }
            }
        }
        
        // Create ROS2 sensor_msgs/Image message
        // For MJPEG: send compressed JPEG data with "jpeg" encoding
        // For RGB3: send raw RGB data with "rgb8" encoding
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();
        
        let data_size = image_data.len();
        let (encoding, step) = if format_is_mjpeg {
            ("jpeg".to_string(), 0u32) // MJPEG: compressed, no step
        } else {
            ("rgb8".to_string(), (width * 3) as u32) // RGB3: 3 bytes per pixel
        };
        
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
            encoding,
            is_bigendian: 0,
            step,
            data: image_data.to_vec(),
        };
        
        // Send message through channel
        if let Err(e) = tx.send(msg).await {
            eprintln!("Failed to send message: {:?}", e);
            // Continue anyway
        }
        
        // Log frame capture info periodically
        if frame_count % 30 == 0 {
            if format_is_mjpeg {
                println!("Published frame #{}: {}x{} MJPEG ({} bytes compressed)", 
                    frame_count,
                    width, height, 
                    data_size);
            } else {
                println!("Published frame #{}: {}x{} RGB3/rgb24 ({} bytes)", 
                    frame_count,
                    width, height, 
                    data_size);
            }
        }
        
        // Note: stream.next() automatically re-queues the buffer when called again,
        // so no manual re-queuing is needed
    }
}
