use libcamera::camera_manager::CameraManager;
use libcamera::stream::StreamRole;
use libcamera::geometry::Size;
use libcamera::formats;
use libcamera::framebuffer_allocator::FrameBufferAllocator;
use image::{ImageBuffer, Rgb, RgbImage};
use std::time::Duration;

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
    // Note: The FrameBufferAllocator API in libcamera-rs 0.6.0 appears to be different
    // The allocate() and buffers() methods don't exist
    // Let's try a different approach - maybe buffers are allocated automatically
    // or we need to use the allocator differently
    let _allocator = FrameBufferAllocator::new(&camera);
    
    // Check if any buffers are allocated (allocated() takes no arguments)
    let is_allocated = _allocator.allocated();
    println!("Buffers allocated: {}", is_allocated);
    
    // Since allocate() and buffers() methods don't exist, we'll try to work
    // without explicit buffer allocation - maybe the camera handles it automatically
    // or we need to use a different API
    println!("Note: FrameBufferAllocator.allocate() and .buffers() methods not found");
    println!("  Attempting to use camera without explicit buffer allocation");
    println!("  Buffers may be allocated automatically or via different API");
    
    // Start the camera
    camera.start(None)
        .map_err(|e| CameraError::CameraStart(format!("Failed to start camera: {:?}", e)))?;
    
    println!("Camera started, attempting to capture a frame...");
    
    // Try to create a request without explicit buffers
    // Maybe the camera/request API handles buffer allocation automatically
    let mut request = camera.create_request(None)
        .ok_or(CameraError::Request("Failed to create request".to_string()))?;
    
    // Try to queue request without adding buffer - see what error we get
    // This will help us understand if buffers are needed and how to get them
    match camera.queue_request(request) {
        Ok(_) => {
            println!("Request queued successfully (without explicit buffer)");
        }
        Err(e) => {
            println!("Failed to queue request without buffer: {:?}", e);
            println!("  This suggests we need to add buffers to requests");
            println!("  But FrameBufferAllocator API needs to be determined");
            return Err(CameraError::Request(format!("Need buffers but allocation API unknown: {:?}", e)));
        }
    }
    
    println!("Request queued, waiting for frame...");
    
    // Wait a bit for frame capture
    std::thread::sleep(Duration::from_millis(500));
    
    println!("Note: Complete implementation requires:");
    println!("  1. Finding FrameBufferAllocator API to allocate/get buffers");
    println!("  2. Adding buffers to requests");
    println!("  3. Waiting for request completion");
    println!("  4. Getting frame data from completed request");
    println!("  5. Converting NV12 to RGB");
    println!("  6. Saving as image file");
    
    println!("\nTo find the correct API, check:");
    println!("  - libcamera-rs 0.6.0 source code on GitHub");
    println!("  - Example code (e.g., jpeg_capture example)");
    println!("  - libcamera C++ API documentation (Rust bindings should mirror it)");
    
    Ok(())
}
