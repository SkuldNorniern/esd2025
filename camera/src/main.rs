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
    // Based on C++ libcamera API: allocator->allocate(stream) and allocator->buffers(stream)
    // In Rust, these should be: allocator.allocate(&stream) and allocator.buffers(&stream)
    let mut allocator = FrameBufferAllocator::new(&camera);
    
    // Allocate buffers for the stream
    // C++: int ret = allocator->allocate(cfg.stream()); (returns < 0 on error)
    // Rust equivalent: should return Result<usize> or Result<()>
    println!("Allocating buffers for stream...");
    
    // Try allocate - the exact return type may vary
    // C++ returns int (< 0 on error), Rust might return Result<usize> or Result<()>
    // Try without type annotation first to see what the compiler says
    allocator.allocate(&stream)
        .map_err(|e| CameraError::Configuration(format!("Failed to allocate buffers: {:?}", e)))?;
    
    println!("Buffers allocated successfully");
    
    // Get allocated buffers
    // C++: const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    // Rust equivalent should return &[FrameBuffer] or Vec<FrameBuffer> or similar
    println!("Retrieving allocated buffers...");
    let buffers = allocator.buffers(&stream);
    println!("Retrieved {} buffer(s)", buffers.len());
    
    if buffers.is_empty() {
        return Err(CameraError::Configuration("No buffers allocated".to_string()));
    }
    
    // Start the camera before queuing requests
    camera.start(None)
        .map_err(|e| CameraError::CameraStart(format!("Failed to start camera: {:?}", e)))?;
    
    println!("Camera started, capturing a frame...");
    
    // Create a request and add a buffer to it
    // C++: request->addBuffer(stream, buffer.get());
    let mut request = camera.create_request(None)
        .ok_or(CameraError::Request("Failed to create request".to_string()))?;
    
    // Add buffer to request (use first buffer)
    // C++: int ret = request->addBuffer(stream, buffer.get());
    let buffer = &buffers[0];
    request.add_buffer(&stream, buffer)
        .map_err(|e| CameraError::Request(format!("Failed to add buffer to request: {:?}", e)))?;
    
    // Queue the request
    // C++: camera->queueRequest(request.get());
    camera.queue_request(request)
        .map_err(|e| CameraError::Request(format!("Failed to queue request: {:?}", e)))?;
    
    println!("Request queued, waiting for frame completion...");
    
    // Wait for request completion
    // In C++, this is handled via signals/slots (camera->requestCompleted.connect(callback))
    // In Rust, we need to find the equivalent API
    std::thread::sleep(Duration::from_millis(500));
    
    println!("Note: Request completion handling needs to be implemented");
    println!("  In C++: camera->requestCompleted.connect(callback)");
    println!("  In Rust: Need to find equivalent signal/callback API");
    println!("  Once request completes, extract frame data and save as image");
    
    Ok(())
}
