// Ball Detection Node - YOLO-based detection using ort-rs and ROS2
// Based on: https://medium.com/@alfred.weirich/rust-ort-onnx-real-time-yolo-on-a-live-webcam-part-1-b6edfb50bf9b
use ros2_client::{Context, NodeOptions, Message, NodeName, MessageTypeName, Name};
use ros2_client::rustdds::QosPolicies;
use std::time::Duration;
use std::path::Path;
use image::{DynamicImage, ImageBuffer, Rgb};
use ndarray::Array4;
use ort::{
    session::builder::SessionBuilder,
    value::Value,
};

// Error type for ball detection operations
#[derive(Debug)]
enum BallDetectError {
    Ros2(String),
    Image(String),
    Detection(String),
    Ort(String),
    Io(String),
}

impl std::fmt::Display for BallDetectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BallDetectError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            BallDetectError::Image(msg) => write!(f, "Image error: {}", msg),
            BallDetectError::Detection(msg) => write!(f, "Detection error: {}", msg),
            BallDetectError::Ort(msg) => write!(f, "ONNX Runtime error: {}", msg),
            BallDetectError::Io(msg) => write!(f, "IO error: {}", msg),
        }
    }
}

impl std::error::Error for BallDetectError {}

// YOLO-based ball detector using ONNX Runtime
struct BallDetector {
    session: ort::session::Session,
    input_shape: (usize, usize), // (height, width) for model input
    confidence_threshold: f32,
}

impl BallDetector {
    fn new(model_path: &str, confidence_threshold: f32) -> Result<Self, BallDetectError> {
        let model_path = Path::new(model_path);
        if !model_path.exists() {
            return Err(BallDetectError::Ort(format!(
                "Model file not found: {}. Export your YOLO model to ONNX format first.",
                model_path.display()
            )));
        }

        // Initialize ONNX Runtime session
        let session = SessionBuilder::new()
            .map_err(|e| BallDetectError::Ort(format!("Failed to create session builder: {:?}", e)))?
            .commit_from_file(model_path)
            .map_err(|e| BallDetectError::Ort(format!("Failed to load ONNX model: {:?}", e)))?;

        // YOLO models typically expect input shape [1, 3, 640, 640]
        let input_shape = (640, 640);

        Ok(Self {
            session,
            input_shape,
            confidence_threshold,
        })
    }

    // Detect ball in image using YOLO
    // Input: image data, width, height, encoding
    // Output: bounding box (x1, y1, x2, y2) or None
    fn detect(
        &mut self,
        image_data: &[u8],
        width: u32,
        height: u32,
        encoding: &str,
    ) -> Result<Option<(f32, f32, f32, f32)>, BallDetectError> {
        // Convert ROS2 image to RGB image buffer
        let rgb_image = match encoding {
            "rgb8" => {
                ImageBuffer::<Rgb<u8>, _>::from_raw(width, height, image_data.to_vec())
                    .ok_or_else(|| BallDetectError::Image("Failed to create RGB image".to_string()))?
            }
            "bgr8" => {
                // BGR to RGB conversion
                let mut rgb_data = Vec::with_capacity((width * height * 3) as usize);
                for chunk in image_data.chunks_exact(3) {
                    rgb_data.push(chunk[2]); // R
                    rgb_data.push(chunk[1]); // G
                    rgb_data.push(chunk[0]); // B
                }
                ImageBuffer::<Rgb<u8>, _>::from_raw(width, height, rgb_data)
                    .ok_or_else(|| BallDetectError::Image("Failed to create RGB image from BGR".to_string()))?
            }
            _ => {
                return Err(BallDetectError::Image(format!(
                    "Unsupported encoding: {}",
                    encoding
                )));
            }
        };

        let dynamic_image = DynamicImage::ImageRgb8(rgb_image);

        // Resize image to model input size
        let resized = dynamic_image.resize_exact(
            self.input_shape.1 as u32,
            self.input_shape.0 as u32,
            image::imageops::FilterType::Triangle,
        );

        // Convert image to normalized tensor [1, 3, H, W] with values in [0, 1]
        let input_size_usize = self.input_shape.0;
        let mut input_array = Array4::<f32>::zeros((1, 3, input_size_usize, self.input_shape.1));
        
        for (y, row) in resized.to_rgb8().rows().enumerate() {
            for (x, pixel) in row.enumerate() {
                let r = pixel[0] as f32 / 255.0;
                let g = pixel[1] as f32 / 255.0;
                let b = pixel[2] as f32 / 255.0;
                
                input_array[[0, 0, y, x]] = r;
                input_array[[0, 1, y, x]] = g;
                input_array[[0, 2, y, x]] = b;
            }
        }

        // Run inference
        let input_tensor = Value::from_array(input_array.into_dyn())
            .map_err(|e| BallDetectError::Ort(format!("Failed to create input tensor: {:?}", e)))?;

        let inputs = ort::inputs!["images" => input_tensor];
        let outputs = self.session
            .run(inputs)
            .map_err(|e| BallDetectError::Ort(format!("YOLO inference failed: {:?}", e)))?;

        // Extract output - returns (shape, data) tuple
        let (output_shape, output_data) = outputs["output0"]
            .try_extract_tensor::<f32>()
            .map_err(|e| BallDetectError::Ort(format!("Failed to extract output: {:?}", e)))?;

        // Parse detections
        let shape_dims: Vec<usize> = output_shape.iter().map(|&d| d as usize).collect();
        if shape_dims.len() != 3 {
            return Err(BallDetectError::Detection(format!(
                "Unexpected output shape: {:?}, expected 3D tensor",
                shape_dims
            )));
        }

        let batch_size = shape_dims[0];
        let num_outputs = shape_dims[1];
        let num_anchors = shape_dims[2];

        if batch_size != 1 {
            return Err(BallDetectError::Detection(format!(
                "Expected batch size 1, got {}",
                batch_size
            )));
        }

        if num_outputs < 5 {
            return Err(BallDetectError::Detection(format!(
                "Expected at least 5 outputs (4 bbox + 1 class), got {}",
                num_outputs
            )));
        }

        // Find best ball detection (class 0)
        let mut best_detection: Option<(f32, f32, f32, f32)> = None;
        let mut best_confidence = 0.0;

        // Access data as flat array and calculate indices manually
        for anchor_idx in 0..num_anchors {
            let base_idx = anchor_idx * num_outputs;
            let center_x = output_data[base_idx + 0];
            let center_y = output_data[base_idx + 1];
            let bbox_width = output_data[base_idx + 2];
            let bbox_height = output_data[base_idx + 3];
            let class_confidence = output_data[base_idx + 4];

            if class_confidence >= self.confidence_threshold && class_confidence > best_confidence {
                // Coordinates are already in pixel space for the model input (640x640)
                // Scale from model input size to original image size
                let scale_x = width as f32 / self.input_shape.1 as f32;
                let scale_y = height as f32 / self.input_shape.0 as f32;

                // Use coordinates directly (they're already in pixels for 640x640)
                let abs_center_x = center_x;
                let abs_center_y = center_y;
                let abs_width = bbox_width;
                let abs_height = bbox_height;

                // Convert to (x1, y1, x2, y2) format in model input space
                let x1_model = abs_center_x - abs_width / 2.0;
                let y1_model = abs_center_y - abs_height / 2.0;
                let x2_model = abs_center_x + abs_width / 2.0;
                let y2_model = abs_center_y + abs_height / 2.0;

                // Scale to original image coordinates
                let x1 = x1_model * scale_x;
                let y1 = y1_model * scale_y;
                let x2 = x2_model * scale_x;
                let y2 = y2_model * scale_y;

                // Clamp to image boundaries
                let x1 = x1.max(0.0).min(width as f32);
                let y1 = y1.max(0.0).min(height as f32);
                let x2 = x2.max(0.0).min(width as f32);
                let y2 = y2.max(0.0).min(height as f32);

                best_detection = Some((x1, y1, x2, y2));
                best_confidence = class_confidence;
            }
        }

        Ok(best_detection)
    }
}

fn main() -> Result<(), BallDetectError> {
    println!("Ball Detection Node (YOLO-based with ONNX Runtime)");
    println!("====================================================");
    println!("This node runs on a more powerful device than the Raspberry Pi");
    println!("It subscribes to /image from the Pi and publishes bounding box coordinates");
    println!();

    // Configure ROS2 to connect to RPi3
    let rpi_ip = "100.114.136.109";
    println!("Target Raspberry Pi IP: {}", rpi_ip);
    println!("Note: Ensure ROS2 is configured for distributed communication");
    println!("  - Both machines should use the same ROS_DOMAIN_ID");
    println!("  - Or configure ROS_DISCOVERY_SERVER if using discovery server");
    println!();

    // Get model path from environment or use default
    let model_path = std::env::var("YOLO_MODEL_PATH")
        .unwrap_or_else(|_| "../yolo/runs/train/tennis_ball_tracker/weights/best.onnx".to_string());
    let confidence_threshold = std::env::var("YOLO_CONFIDENCE_THRESHOLD")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.25);

    println!("Loading YOLO ONNX model from: {}", model_path);
    println!("Confidence threshold: {:.2}", confidence_threshold);
    println!();

    let mut detector = BallDetector::new(&model_path, confidence_threshold)?;
    println!("YOLO model loaded successfully");
    println!();

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create ROS2 context: {:?}", e)))?;

    // Create node using Context::new_node
    let node_name = NodeName::new("", "ball_detect_node")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create node name: {:?}", e)))?;
    let node = ctx
        .new_node(node_name, NodeOptions::new().enable_rosout(true))
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create ROS2 node: {:?}", e)))?;

    println!("ROS2 node created: ball_detect_node");
    println!("ROS_DOMAIN_ID: {:?}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!();

    // Create subscriber for /image topic from RPi3
    // TODO: Fix subscription creation - needs type annotations
    // Subscription will be created once message API is clarified
    println!("Subscribing to /image topic from RPi3...");
    println!("(Subscription creation commented out until ROS2 message API is fixed)");

    // Create publishers for bounding box coordinates
    // TODO: Fix publisher creation - needs type annotations
    // Publishers will be created once message API is clarified
    println!("Publishers will be created once ROS2 message API is fixed");
    println!("  /ball_x1 - top-left x coordinate");
    println!("  /ball_y1 - top-left y coordinate");
    println!("  /ball_x2 - bottom-right x coordinate");
    println!("  /ball_y2 - bottom-right y coordinate");
    println!();

    println!("Waiting for images on /image topic from RPi3 at {}...", rpi_ip);
    println!("(Press Ctrl+C to stop)");
    println!();

    // Main processing loop
    // TODO: Fix ROS2 message API - Message trait doesn't have new() method
    // Need to use concrete message types or find correct API for dynamic message creation
    println!("ROS2 node initialized. Message receiving/publishing needs API fixes.");
    println!("For now, use test_yolo binary to test YOLO detection on static images:");
    println!("  cargo run --bin test_yolo --release");
    println!();
    println!("Waiting for ROS2 API fixes...");
    
    loop {
        std::thread::sleep(Duration::from_secs(1));
        // TODO: Implement message receiving and publishing once API is clarified
        // The ros2-client Message trait doesn't support Message::new() directly
        // Need to use concrete message types or find the correct dynamic message API
    }
}
