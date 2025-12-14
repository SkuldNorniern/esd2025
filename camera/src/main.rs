use libcamera::camera_manager::CameraManager;
use libcamera::stream::StreamRole;
use libcamera::pixel_format::PixelFormat;
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
    let mut config = camera.generate_configuration(&[StreamRole::VideoRecording])
        .map_err(|e| CameraError::Configuration(format!("Failed to generate configuration: {:?}", e)))?;

    // Configure stream settings
    if let Some(stream_config) = config.streams_mut().get_mut(0) {
        // Set resolution to 320x240 for low latency
        stream_config.size = Size::new(320, 240);
        // Use NV12 format (common for video, similar to YUV420)
        // PixelFormat from predefined formats module
        stream_config.pixel_format = formats::NV12;
    }

    // Validate and apply configuration
    config.validate();
    camera.configure(&config)
        .map_err(|e| CameraError::Configuration(format!("Failed to configure camera: {:?}", e)))?;

    println!("Camera configured: {:?}", config.streams()[0].size);

    // Allocate buffers and create requests
    let mut active_camera = camera.start()
        .map_err(|e| CameraError::CameraStart(format!("Failed to start camera: {:?}", e)))?;

    // Create multiple requests for continuous capture
    let num_requests = 4;
    let mut requests = Vec::new();
    for _ in 0..num_requests {
        let request = active_camera.create_request()
            .map_err(|e| CameraError::Request(format!("Failed to create request: {:?}", e)))?;
        active_camera.queue_request(request)
            .map_err(|e| CameraError::Request(format!("Failed to queue request: {:?}", e)))?;
    }

    println!("Camera started, capturing frames...");
    println!("(Press Ctrl+C to stop)");

    // Main capture loop
    let mut frame_count = 0u64;
    loop {
        // Wait for completed request
        let completed_request = active_camera.wait_for_request(Duration::from_secs(1))
            .map_err(|e| CameraError::Frame(format!("Failed to wait for request: {:?}", e)))?;

        // Process the frame
        if let Some(buffer) = completed_request.buffers().get(0) {
            let stream_config = &config.streams()[0];
            let width = stream_config.size.width;
            let height = stream_config.size.height;

            // Get frame data
            // Note: The exact API depends on libcamera-rs version
            // For now, just log that we received a frame
            // The actual plane access API may vary
            frame_count += 1;
            
            // Log frame info periodically
            if frame_count == 1 {
                println!("First frame captured: {}x{}", width, height);
                println!("  Format: {:?}", stream_config.pixel_format);
                println!("  Buffer received (plane access API may vary by libcamera-rs version)");
            }
            
            if frame_count % 30 == 0 {
                println!("Captured frame #{}: {}x{}", 
                    frame_count, width, height);
            }
        }

        // Re-queue the request for next frame
        active_camera.queue_request(completed_request)
            .map_err(|e| CameraError::Request(format!("Failed to re-queue request: {:?}", e)))?;
    }
}
