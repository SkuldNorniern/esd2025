// Ball Detection Node - YOLO-based detection using ort-rs and ROS2
// Uses ros_wrapper for ROS communication and receives PNG images via sensor_msgs/Image
use ros_wrapper::{create_topic_receiver, QosProfile, sensor_msgs::msg::Image};
use std::time::{Duration, Instant};
use image::{DynamicImage, ImageBuffer, Rgb};
use ndarray::Array4;
use ort::{
    session::builder::SessionBuilder,
    value::Value,
};
use std::path::Path;

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
    // Input: PNG image data
    // Output: bounding box (x1, y1, x2, y2) or None
    fn detect(
        &mut self,
        png_data: &[u8],
    ) -> Result<Option<(f32, f32, f32, f32)>, BallDetectError> {
        // Decode PNG image
        let img = image::load_from_memory(png_data)
            .map_err(|e| BallDetectError::Image(format!("Failed to decode PNG: {:?}", e)))?;
        
        let (width, height) = (img.width(), img.height());
        let rgb_image = img.to_rgb8();

        // Resize image to model input size
        let resized = DynamicImage::ImageRgb8(rgb_image).resize_exact(
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
        // YOLO output format is [batch, outputs, anchors] = [1, 5, 8400]
        // where outputs are [x_center, y_center, width, height, confidence]
        // Coordinates are already in pixel space for the model input size (640x640)
        let mut best_detection: Option<(f32, f32, f32, f32)> = None;
        let mut best_confidence = 0.0;

        // Parse detections using [batch, outputs, anchors] layout
        for anchor_idx in 0..num_anchors {
            // Index calculation: batch * (outputs * anchors) + output * anchors + anchor
            let x_idx = 0 * (num_outputs * num_anchors) + 0 * num_anchors + anchor_idx;
            let y_idx = 0 * (num_outputs * num_anchors) + 1 * num_anchors + anchor_idx;
            let w_idx = 0 * (num_outputs * num_anchors) + 2 * num_anchors + anchor_idx;
            let h_idx = 0 * (num_outputs * num_anchors) + 3 * num_anchors + anchor_idx;
            let conf_idx = 0 * (num_outputs * num_anchors) + 4 * num_anchors + anchor_idx;
            
            let center_x = output_data[x_idx];
            let center_y = output_data[y_idx];
            let bbox_width = output_data[w_idx];
            let bbox_height = output_data[h_idx];
            let class_confidence = output_data[conf_idx];

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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Ball Detection Node (YOLO-based with ONNX Runtime)");
    println!("====================================================");
    println!("This node runs on a more powerful device than the Raspberry Pi");
    println!("It subscribes to /image from the Pi and publishes bounding box coordinates");
    println!();

    // Get model path from environment or use default
    let model_path = std::env::var("YOLO_MODEL_PATH")
        .unwrap_or_else(|_| "./best.onnx".to_string());
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

    // Create topic receiver using ros_wrapper
    println!("Subscribing to /image topic...");
    let (mut rx, _node) = create_topic_receiver::<Image>(
        "ball_detect_node",
        "/image",
        QosProfile::default(),
    )
    .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic receiver: {:?}", e)))?;

    println!("Subscriber created successfully");
    println!("  Topic: /image");
    println!("  Message type: sensor_msgs/Image");
    println!("  Node: ball_detect_node");
    println!("ROS_DOMAIN_ID: {}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!();
    
    // Wait for the subscriber to establish connections with publishers
    println!("Waiting for subscriber to establish connections with publishers...");
    println!("  (DDS discovery can take 5-10 seconds for cross-device communication)");
    for _i in 1..=10 {
        tokio::time::sleep(Duration::from_secs(1)).await;
        print!(".");
        std::io::Write::flush(&mut std::io::stdout())?;
    }
    println!();
    println!("Subscriber ready, waiting for messages...");
    println!("(Press Ctrl+C to stop)");
    println!();

    let mut frame_count = 0u64;
    let mut last_log_time = Instant::now();
    
    // Process incoming messages from channel
    while let Some(msg) = rx.recv().await {
        frame_count += 1;
        
        if frame_count == 1 {
            println!("✓ First message received!");
        }
        
        // Check encoding - should be "png"
        if msg.encoding != "png" {
            eprintln!("Frame #{}: Unexpected encoding '{}', expected 'png'", frame_count, msg.encoding);
            continue;
        }
        
        // Run detection on PNG data
        match detector.detect(&msg.data) {
            Ok(Some((x1, y1, x2, y2))) => {
                println!("Frame #{}: Ball detected at ({:.1}, {:.1}) to ({:.1}, {:.1})", 
                    frame_count, x1, y1, x2, y2);
            }
            Ok(None) => {
                if frame_count % 30 == 0 {
                    println!("Frame #{}: No ball detected", frame_count);
                }
            }
            Err(e) => {
                eprintln!("Detection error on frame #{}: {}", frame_count, e);
            }
        }
        
        // Log periodically if no messages received yet
        if frame_count == 0 {
            let now = Instant::now();
            if now.duration_since(last_log_time) > Duration::from_secs(2) {
                println!("Waiting for messages...");
                last_log_time = now;
            }
        }
    }

    Ok(())
}
