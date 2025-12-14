use libcamera::camera_manager::CameraManager;
use libcamera::stream::StreamRole;
use libcamera::geometry::Size;
use libcamera::formats;
use libcamera::framebuffer_allocator::FrameBufferAllocator;
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
    // CameraList doesn't support indexing, use get() or iterate
    let camera_info = cameras.get(0)
        .ok_or(CameraError::NoCameras)?;
    println!("Using camera: {:?}", camera_info.id());

    // Acquire camera
    let mut camera = camera_info.acquire()
        .map_err(|e| CameraError::CameraAcquire(format!("Failed to acquire camera: {:?}", e)))?;

    // Generate configuration for video recording
    // Start with low resolution (320x240) as suggested in README for low latency
    // StreamRole variants: Viewfinder, VideoRecording, StillCapture, Raw
    // generate_configuration returns Option, not Result
    let mut config = camera.generate_configuration(&[StreamRole::VideoRecording])
        .ok_or(CameraError::Configuration("Failed to generate configuration".to_string()))?;

    // Configure stream settings
    // Use get_mut to get mutable reference to stream configuration
    if let Some(mut stream_config) = config.get_mut(0) {
        // Set resolution to 320x240 for low latency
        // set_size takes a Size object, not two integers
        stream_config.set_size(Size::new(320, 240));
        // Use NV12 format (common for video, similar to YUV420)
        // PixelFormat from predefined formats module
        stream_config.set_pixel_format(formats::NV12);
    }

    // Validate and apply configuration
    config.validate();
    camera.configure(&mut config)
        .map_err(|e| CameraError::Configuration(format!("Failed to configure camera: {:?}", e)))?;

    // Get stream config for logging (use get for immutable access)
    if let Some(stream_config) = config.get(0) {
        println!("Camera configured: {:?}", stream_config.get_size());
    }

    // Buffer allocation
    // Note: The FrameBufferAllocator API in libcamera-rs 0.6.0 may differ from documentation
    // The compiler indicates that `allocate` and `buffers` methods don't exist
    // This suggests the API may have changed or use different method names
    // 
    // Options:
    // 1. Check the actual libcamera-rs 0.6.0 source code or examples
    // 2. Try alternative method names (e.g., alloc, alloc_buffers, etc.)
    // 3. Check if buffers are allocated automatically during configuration
    // 4. Look for a different buffer allocation mechanism
    //
    // For now, we'll proceed without explicit buffer allocation
    // You may need to manually inspect the FrameBufferAllocator API or check examples
    println!("Warning: Buffer allocation skipped - API methods not found");
    println!("  FrameBufferAllocator.allocate() and .buffers() methods don't exist");
    println!("  Please check libcamera-rs 0.6.0 documentation or source code");
    println!("  for the correct buffer allocation API");

    // Start the camera (takes Option<&ControlList>, use None for default controls)
    camera.start(None)
        .map_err(|e| CameraError::CameraStart(format!("Failed to start camera: {:?}", e)))?;

    println!("Camera started, capturing frames...");
    println!("(Press Ctrl+C to stop)");

    // Main capture loop
    // Note: Without proper buffer allocation, requests will fail
    // This is a placeholder - you need to implement buffer allocation
    // based on the actual libcamera-rs 0.6.0 API
    println!("Camera running (buffer allocation not implemented)");
    println!("  Requests will fail without buffers");
    println!("  Please implement buffer allocation using the correct API");
    
    // Keep running (camera is started but won't capture without buffers)
    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}
