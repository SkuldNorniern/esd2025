use libcamera::camera_manager::CameraManager;
use libcamera::stream::StreamRole;
use libcamera::geometry::Size;
use libcamera::formats;
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
    let cm = CameraManager::new()
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
        stream_config.set_size(320, 240);
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
        println!("Camera configured: {:?}", stream_config.size());
    }

    // Start the camera (takes Option<&ControlList>, use None for default controls)
    camera.start(None)
        .map_err(|e| CameraError::CameraStart(format!("Failed to start camera: {:?}", e)))?;

    println!("Camera started, capturing frames...");
    println!("(Press Ctrl+C to stop)");

    // Main capture loop
    let mut frame_count = 0u64;
    loop {
        // Create a capture request
        // create_request returns Option, not Result
        let request = camera.create_request(None)
            .ok_or(CameraError::Request("Failed to create request".to_string()))?;

        // Queue the request
        camera.queue_request(request)
            .map_err(|e| CameraError::Request(format!("Failed to queue request: {:?}", e)))?;

        // Wait for completed request
        // Note: The exact API for waiting may vary - this is a simplified version
        // You may need to use a different method to wait for request completion
        std::thread::sleep(Duration::from_millis(33)); // ~30 FPS
        
        frame_count += 1;
        
        // Log frame info periodically
        if frame_count == 1 {
            if let Some(stream_config) = config.get(0) {
                let size = stream_config.size();
                println!("First frame queued: {}x{}", size.width, size.height);
                println!("  Format: {:?}", stream_config.pixel_format());
                println!("  Note: Frame processing API may vary by libcamera-rs version");
            }
        }
        
        if frame_count % 30 == 0 {
            if let Some(stream_config) = config.get(0) {
                let size = stream_config.size();
                println!("Queued frame #{}: {}x{}", 
                    frame_count, size.width, size.height);
            }
        }
    }
}
