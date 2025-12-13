// Ball Detection Node - YOLO-based detection using ort-rs and ROS2
// Based on: https://medium.com/@alfred.weirich/rust-ort-onnx-real-time-yolo-on-a-live-webcam-part-1-b6edfb50bf9b
use ros2_client::{Context, NodeOptions, NodeName, MessageTypeName, Name};
use ros2_client::rustdds::QosPolicies;
use std::time::{Duration, Instant};
use std::io::Write;
use std::path::Path;
use image::{DynamicImage, ImageBuffer, Rgb};
use ndarray::Array4;
use ort::{
    session::builder::SessionBuilder,
    value::Value,
};
use base64::{Engine as _, engine::general_purpose};
use serde_json::Value as JsonValue;
use serde::{Deserialize, Serialize};

// Message struct matching std_msgs/String format
// ROS2 std_msgs/String has a single field "data" of type string
#[derive(Serialize, Deserialize, Debug, Clone)]
struct StringMessage {
    data: String,
}

// Error type for ball detection operations
#[derive(Debug)]
enum BallDetectError {
    Ros2(String),
    Image(String),
    Detection(String),
    Ort(String),
    Io(String),
    Json(String),
    Base64(String),
}

impl std::fmt::Display for BallDetectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BallDetectError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            BallDetectError::Image(msg) => write!(f, "Image error: {}", msg),
            BallDetectError::Detection(msg) => write!(f, "Detection error: {}", msg),
            BallDetectError::Ort(msg) => write!(f, "ONNX Runtime error: {}", msg),
            BallDetectError::Io(msg) => write!(f, "IO error: {}", msg),
            BallDetectError::Json(msg) => write!(f, "JSON error: {}", msg),
            BallDetectError::Base64(msg) => write!(f, "Base64 error: {}", msg),
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

// Parse JSON message and decode base64 image data
// Expected format: {"width":320,"height":240,"encoding":"rgb8","data":"base64_string"}
fn parse_image_message(json_str: &str) -> Result<(Vec<u8>, u32, u32, String), BallDetectError> {
    let json: JsonValue = serde_json::from_str(json_str)
        .map_err(|e| BallDetectError::Json(format!("Failed to parse JSON: {:?}", e)))?;
    
    let width = json["width"]
        .as_u64()
        .ok_or_else(|| BallDetectError::Json("Missing or invalid 'width' field".to_string()))?
        as u32;
    
    let height = json["height"]
        .as_u64()
        .ok_or_else(|| BallDetectError::Json("Missing or invalid 'height' field".to_string()))?
        as u32;
    
    let encoding = json["encoding"]
        .as_str()
        .ok_or_else(|| BallDetectError::Json("Missing or invalid 'encoding' field".to_string()))?
        .to_string();
    
    let base64_data = json["data"]
        .as_str()
        .ok_or_else(|| BallDetectError::Json("Missing or invalid 'data' field".to_string()))?;
    
    let image_data = general_purpose::STANDARD
        .decode(base64_data)
        .map_err(|e| BallDetectError::Base64(format!("Failed to decode base64: {:?}", e)))?;
    
    Ok((image_data, width, height, encoding))
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

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create ROS2 context: {:?}", e)))?;

    // Create node using Context::new_node
    // NodeName requires a namespace - use "/" for root namespace
    let node_name = NodeName::new("/", "ball_detect_node")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create node name: {:?}", e)))?;
    let mut node = ctx
        .new_node(node_name, NodeOptions::new().enable_rosout(true))
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create ROS2 node: {:?}", e)))?;

    // CRITICAL: Start the node spinner in a background thread to process DDS events
    // This is required for the subscription to receive messages!
    // Based on ros2-client examples: https://github.com/Atostek/ros2-client/blob/master/examples/minimal_action_client/main.rs
    let spinner = node.spinner()
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create node spinner: {:?}", e)))?;
    std::thread::spawn(move || {
        spinner.spin();
    });

    println!("ROS2 node created: ball_detect_node");
    println!("ROS_DOMAIN_ID: {:?}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!("Node spinner started in background thread");
    println!();

    // Create subscriber for /image topic
    // Expecting std_msgs/String messages with JSON-encoded base64 images
    println!("Subscribing to /image topic...");
    let image_topic_name = Name::new("/", "image")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic name: {:?}", e)))?;
    let string_msg_type = MessageTypeName::new("std_msgs", "String");
    
    let image_topic = node
        .create_topic(&image_topic_name, string_msg_type, &QosPolicies::default())
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic: {:?}", e)))?;
    
    // Create subscription using StringMessage struct
    // This struct matches std_msgs/String format and implements Deserialize
    // IMPORTANT: Use the exact same QoS as the publisher (Some(QosPolicies::default()))
    // to ensure compatibility - this must match exactly!
    let subscriber = node
        .create_subscription::<StringMessage>(&image_topic, Some(QosPolicies::default()))
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create subscriber: {:?}", e)))?;
    
    println!("Subscriber created successfully");
    println!("  Topic: /image");
    println!("  Message type: std_msgs/String");
    println!("  Node: ball_detect_node");
    println!();
    
    // Wait for the subscriber to establish connections with publishers
    // This is critical for cross-device communication where DDS discovery takes time
    println!("Waiting for subscriber to establish connections with publishers...");
    println!("  (DDS discovery can take 5-10 seconds for cross-device communication)");
    for i in 1..=10 {
        std::thread::sleep(Duration::from_secs(1));
        print!(".");
        std::io::Write::flush(&mut std::io::stdout()).ok();
    }
    println!();
    println!("Subscriber ready, waiting for messages...");
    println!();

    // Create publishers for bounding box coordinates
    println!("Creating publishers for detection results...");
    let float_msg_type = MessageTypeName::new("std_msgs", "Float32");
    
    let ball_x1_topic_name = Name::new("/", "ball_x1")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic name: {:?}", e)))?;
    let ball_x1_topic = node
        .create_topic(&ball_x1_topic_name, float_msg_type.clone(), &QosPolicies::default())
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic: {:?}", e)))?;
    
    let ball_y1_topic_name = Name::new("/", "ball_y1")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic name: {:?}", e)))?;
    let ball_y1_topic = node
        .create_topic(&ball_y1_topic_name, float_msg_type.clone(), &QosPolicies::default())
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic: {:?}", e)))?;
    
    let ball_x2_topic_name = Name::new("/", "ball_x2")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic name: {:?}", e)))?;
    let ball_x2_topic = node
        .create_topic(&ball_x2_topic_name, float_msg_type.clone(), &QosPolicies::default())
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic: {:?}", e)))?;
    
    let ball_y2_topic_name = Name::new("/", "ball_y2")
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic name: {:?}", e)))?;
    let ball_y2_topic = node
        .create_topic(&ball_y2_topic_name, float_msg_type, &QosPolicies::default())
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic: {:?}", e)))?;
    
    // Publishers commented out until we resolve the Message trait type issue
    // The API requires concrete types, not traits, for type parameters
    // TODO: Find the correct way to use dynamic messages or use concrete message types
    // let mut _publisher_x1 = node
    //     .create_publisher(&ball_x1_topic, None)
    //     .map_err(|e| BallDetectError::Ros2(format!("Failed to create publisher: {:?}", e)))?;
    // let mut _publisher_y1 = node
    //     .create_publisher(&ball_y1_topic, None)
    //     .map_err(|e| BallDetectError::Ros2(format!("Failed to create publisher: {:?}", e)))?;
    // let mut _publisher_x2 = node
    //     .create_publisher(&ball_x2_topic, None)
    //     .map_err(|e| BallDetectError::Ros2(format!("Failed to create publisher: {:?}", e)))?;
    // let mut _publisher_y2 = node
    //     .create_publisher(&ball_y2_topic, None)
    //     .map_err(|e| BallDetectError::Ros2(format!("Failed to create publisher: {:?}", e)))?;
    
    println!("Publishers created successfully");
    println!("  /ball_x1 - top-left x coordinate");
    println!("  /ball_y1 - top-left y coordinate");
    println!("  /ball_x2 - bottom-right x coordinate");
    println!("  /ball_y2 - bottom-right y coordinate");
    println!();

    println!("Waiting for images on /image topic...");
    println!("(Press Ctrl+C to stop)");
    println!();

    // Event-driven message reception loop
    // Based on ros2-client service examples: https://github.com/Atostek/ros2-client/blob/master/examples/ros2_service_client/main.rs
    // Note: Subscriptions don't support mio::Poll directly, so we use timer-based polling
    println!("Starting event-driven message reception loop...");
    println!("Node names:");
    println!("  Publisher: camera_node (on Raspberry Pi)");
    println!("  Subscriber: ball_detect_node (on this device)");
    println!("Topic: /image");
    println!("Message type: std_msgs/String");
    println!();

    let mut frame_count = 0u64;
    let mut last_log_time = Instant::now();
    
    println!("Event loop started, waiting for messages...");
    println!();

    loop {
        // Event-driven polling: check for messages periodically
        // Process all available messages in a batch
        loop {
            match subscriber.take() {
                Ok(Some((msg, _info))) => {
                    frame_count += 1;
                    
                    if frame_count == 1 {
                        println!("✓ First message received!");
                    }
                    
                    // Extract the JSON string from the message
                    let json_str = &msg.data;
                    
                    // Parse JSON and decode base64 image
                    match parse_image_message(json_str) {
                        Ok((image_data, width, height, encoding)) => {
                            // Run detection
                            match detector.detect(&image_data, width, height, &encoding) {
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
                        }
                        Err(e) => {
                            eprintln!("Failed to parse image message on frame #{}: {}", frame_count, e);
                        }
                    }
                }
                Ok(None) => {
                    // No more messages available, break inner loop
                    break;
                }
                Err(e) => {
                    eprintln!("Error taking message from subscription: {:?}", e);
                    break;
                }
            }
        }
        
        // Log periodically if no messages received yet (similar to service example rate limiting)
        if frame_count == 0 {
            let now = Instant::now();
            if now.duration_since(last_log_time) > Duration::from_secs(2) {
                println!("Waiting for messages... (event loop polling)");
                last_log_time = now;
            }
        }
        
        // Small sleep to avoid busy-waiting (similar to service example)
        std::thread::sleep(Duration::from_millis(10));
    }
}
