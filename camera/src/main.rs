use libcamera::{CameraManager, StreamRole, PixelFormat};
use ros2_client::{Context, Node, NodeOptions, QosProfile};
use std::time::{Duration, SystemTime};

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
    Ros2(String),
    Message(String),
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
            CameraError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            CameraError::Message(msg) => write!(f, "Message error: {}", msg),
        }
    }
}

impl std::error::Error for CameraError {}

fn main() -> Result<(), CameraError> {
    println!("Initializing camera node...");

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| CameraError::Ros2(format!("Failed to create ROS2 context: {:?}", e)))?;
    let node = Node::new(&ctx, "camera_node", &NodeOptions::new().enable_rosout(true))
        .map_err(|e| CameraError::Ros2(format!("Failed to create ROS2 node: {:?}", e)))?;

    // Create publisher for /image topic
    // Note: ros2-client uses message type names as strings
    // sensor_msgs/msg/Image is the standard ROS2 message type
    println!("Creating ROS2 publisher on /image topic...");
    
    // Create publisher with sensor_msgs/msg/Image message type
    // The exact API may vary, but typically it's something like:
    // let publisher = node.create_publisher::<sensor_msgs::msg::Image>(...)
    // For now, we'll set up the structure and capture frames
    // The actual publishing will be implemented once we confirm the message type API

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
        println!("  Camera {}: {}", i, camera.id());
    }

    // Use first available camera
    let camera_info = &cameras[0];
    println!("Using camera: {}", camera_info.id());

    // Acquire camera
    let mut camera = camera_info.acquire()
        .map_err(|e| CameraError::CameraAcquire(format!("Failed to acquire camera: {:?}", e)))?;

    // Generate configuration for video capture
    // Start with low resolution (320x240) as suggested in README for low latency
    let mut config = camera.generate_configuration(&[StreamRole::VideoCapture])
        .map_err(|e| CameraError::Configuration(format!("Failed to generate configuration: {:?}", e)))?;

    // Configure stream settings
    if let Some(stream_config) = config.streams_mut().get_mut(0) {
        // Set resolution to 320x240 for low latency
        stream_config.size = libcamera::Size::new(320, 240);
        // Use YUV420 format (common for video)
        stream_config.pixel_format = PixelFormat::Yuv420;
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
    println!("Publishing to /image topic (press Ctrl+C to stop)");

    // Main capture loop
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
            let planes = buffer.planes();
            if planes.is_empty() {
                eprintln!("Warning: No planes in buffer");
                continue;
            }

            // Convert frame to ROS2 Image message format
            // For YUV420, we need to handle the planar format
            // Collect all plane data into a single buffer
            let mut frame_data = Vec::new();
            for plane in planes.iter() {
                frame_data.extend_from_slice(plane);
            }
            
            // Create ROS2 Image message
            // sensor_msgs/Image structure:
            // - header (std_msgs/Header): stamp, frame_id
            // - height: u32
            // - width: u32
            // - encoding: String (e.g., "yuv420", "rgb8")
            // - is_bigendian: u8
            // - step: u32 (bytes per row)
            // - data: Vec<u8>
            
            // For YUV420 format:
            // - encoding: "yuv420"
            // - step: width (for Y plane, but YUV420 is planar so this is approximate)
            // - data: concatenated Y, U, V planes
            
            // TODO: Create actual ROS2 message using ros2-client API
            // The exact API depends on how ros2-client handles message types
            // Example structure (may need adjustment based on actual API):
            // let image_msg = sensor_msgs::msg::Image {
            //     header: ...,
            //     height,
            //     width,
            //     encoding: "yuv420".to_string(),
            //     is_bigendian: 0,
            //     step: width, // approximate for YUV420
            //     data: frame_data,
            // };
            // publisher.publish(&image_msg)?;
            
            println!("Captured frame: {}x{} ({} bytes) - ready to publish", 
                width, height, 
                frame_data.len());
        }

        // Re-queue the request for next frame
        active_camera.queue_request(completed_request)
            .map_err(|e| CameraError::Request(format!("Failed to re-queue request: {:?}", e)))?;
    }
}
