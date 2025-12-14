use libcamera::camera_manager::CameraManager;
use libcamera::stream::StreamRole;
use libcamera::geometry::Size;
use libcamera::formats;
use libcamera::framebuffer_allocator::FrameBufferAllocator;
use image::{ImageBuffer, Rgb, RgbImage};
use std::time::Duration;
use std::fs::File;
use std::io::Write;

// Error type for camera operations
#[derive(Debug)]
enum CameraError {
    CameraManager(String),
    NoCameras,
    CameraAcquire(String),
    Configuration(String),
    CameraStart(String),
    Request(String),
    Frame(String),
    Io(String),
}

impl std::fmt::Display for CameraError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CameraError::CameraManager(msg) => write!(f, "CameraManager error: {}", msg),
            CameraError::NoCameras => write!(f, "No cameras found"),
            CameraError::CameraAcquire(msg) => write!(f, "Camera acquire error: {}", msg),
            CameraError::Configuration(msg) => write!(f, "Configuration error: {}", msg),
            CameraError::CameraStart(msg) => write!(f, "Camera start error: {}", msg),
            CameraError::Request(msg) => write!(f, "Request error: {}", msg),
            CameraError::Frame(msg) => write!(f, "Frame error: {}", msg),
            CameraError::Io(msg) => write!(f, "IO error: {}", msg),
        }
    }
}

impl std::error::Error for CameraError {}

fn main() -> Result<(), CameraError> {
    println!("Initializing camera with libcamera-rs...");

    // Initialize CameraManager
    let mut cm = CameraManager::new()
        .map_err(|e| CameraError::CameraManager(format!("Failed to create CameraManager: {:?}", e)))?;
    
    cm.start()
        .map_err(|e| CameraError::CameraManager(format!("Failed to start CameraManager: {:?}", e)))?;

    // Enumerate cameras
    let cameras = cm.cameras();
    if cameras.is_empty() {
        return Err(CameraError::NoCameras);
    }

    println!("Found {} camera(s)", cameras.len());
    for (i, camera) in cameras.iter().enumerate() {
        println!("  Camera {}: {:?}", i, camera.id());
    }

    // Use first available camera
    let camera_info = cameras.get(0)
        .ok_or(CameraError::NoCameras)?;
    println!("Using camera: {:?}", camera_info.id());

    // Acquire camera
    let mut camera = camera_info.acquire()
        .map_err(|e| CameraError::CameraAcquire(format!("Failed to acquire camera: {:?}", e)))?;

    // Generate configuration for video recording
    let mut config = camera.generate_configuration(&[StreamRole::VideoRecording])
        .ok_or(CameraError::Configuration("Failed to generate configuration".to_string()))?;

    // Configure stream settings
    if let Some(mut stream_config) = config.get_mut(0) {
        stream_config.set_size(Size::new(320, 240));
        stream_config.set_pixel_format(formats::NV12);
    }

    // Validate and apply configuration
    config.validate();
    camera.configure(&mut config)
        .map_err(|e| CameraError::Configuration(format!("Failed to configure camera: {:?}", e)))?;

    // Get stream config for logging
    if let Some(stream_config) = config.get(0) {
        println!("Camera configured: {:?}", stream_config.get_size());
    }

    // Get the stream from configuration
    let stream = if let Some(stream_config) = config.get(0) {
        stream_config.stream()
            .ok_or(CameraError::Configuration("Stream not available after configuration".to_string()))?
    } else {
        return Err(CameraError::Configuration("No stream configuration available".to_string()));
    };
    
    // Create frame buffer allocator
    let mut allocator = FrameBufferAllocator::new(&camera);
    
    // Check if buffers are already allocated
    let is_allocated = allocator.allocated(&stream);
    println!("Buffers allocated for stream: {}", is_allocated);
    
    // Try to allocate buffers - the method signature might require &mut self
    // or the method name might be different
    if !is_allocated {
        println!("Allocating buffers...");
        // Try allocate with mutable reference - common in Rust APIs
        // If this doesn't compile, the method name or signature is different
        let num_buffers = allocator.allocate(&stream)
            .map_err(|e| CameraError::Configuration(format!("Failed to allocate buffers: {:?}", e)))?;
        println!("Allocated {} buffers", num_buffers);
    }
    
    // Get allocated buffers
    // Try buffers() method - if it doesn't exist, we'll get a compile error
    // that will help identify the correct method name
    let buffers = allocator.buffers(&stream);
    println!("Retrieved {} buffer(s)", buffers.len());
    
    if buffers.is_empty() {
        return Err(CameraError::Configuration("No buffers allocated".to_string()));
    }
    
    // Start the camera
    camera.start(None)
        .map_err(|e| CameraError::CameraStart(format!("Failed to start camera: {:?}", e)))?;
    
    println!("Camera started, attempting to capture a frame...");
    
    // Capture a single frame and save it
    let mut request = camera.create_request(None)
        .ok_or(CameraError::Request("Failed to create request".to_string()))?;
    
    // Add buffer to request
    let buffer = &buffers[0];
    request.add_buffer(&stream, buffer)
        .map_err(|e| CameraError::Request(format!("Failed to add buffer: {:?}", e)))?;
    
    // Queue request
    camera.queue_request(request)
        .map_err(|e| CameraError::Request(format!("Failed to queue request: {:?}", e)))?;
    
    println!("Request queued, waiting for frame...");
    
    // TODO: Wait for request completion and get frame data
    // The exact API for waiting for request completion needs to be determined
    // Common patterns:
    // - camera.wait_for_request() or similar
    // - Request completion callback
    // - Polling request status
    
    // For now, wait a bit and then try to save
    std::thread::sleep(Duration::from_millis(100));
    
    println!("Note: Frame data retrieval not yet implemented");
    println!("  Need to determine libcamera-rs 0.6.0 API for:");
    println!("  1. Waiting for request completion");
    println!("  2. Getting frame buffer data from completed request");
    println!("  3. Converting NV12 format to RGB");
    println!("  4. Saving as image file");
    
    // Once frame data is available, convert NV12 to RGB and save:
    /*
    // Get frame data from buffer (API needs to be determined)
    let frame_data = /* get data from buffer */;
    
    // Convert NV12 to RGB (NV12 is YUV 4:2:0 format)
    // This requires YUV to RGB conversion
    let rgb_data = convert_nv12_to_rgb(frame_data, 320, 240);
    
    // Create image and save
    let img: RgbImage = ImageBuffer::<Rgb<u8>, _>::from_raw(320, 240, rgb_data)
        .ok_or(CameraError::Frame("Failed to create image".to_string()))?;
    
    img.save("captured_image.png")
        .map_err(|e| CameraError::Io(format!("Failed to save image: {:?}", e)))?;
    
    println!("Image saved as captured_image.png");
    */
    
    Ok(())
}
