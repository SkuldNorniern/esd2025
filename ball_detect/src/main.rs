// Ball Detection Node - YOLO-based detection using ort-rs and ROS2
// Uses ros_wrapper for ROS communication and receives RGB3/rgb24 (24-bit RGB 8-8-8) or PNG images via sensor_msgs/Image
// RGB3 (V4L2) / rgb24 (ffmpeg): Stepwise 16x16 - 16376x16376 with step 1/1
// Publishes ball coordinates as a single string message: "x1,y1,x2,y2" or "none"
mod image_utils;

use ros_wrapper::{create_topic_receiver, create_topic_sender, QosProfile, sensor_msgs::msg::Image, std_msgs::msg::String as StringMsg};
use std::time::{Duration, Instant};
use std::path::Path;
use std::sync::{Arc, Mutex};
use image::{DynamicImage, Rgb, RgbImage};
use ndarray::Array4;
use ort::{
    session::builder::SessionBuilder,
    value::Value,
};
use image_utils::{rgb8_to_png, draw_bbox};
use eframe::egui;

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

// Shared state for GUI display
struct GuiState {
    image: Option<RgbImage>,
    ball_bbox: Option<(f32, f32, f32, f32)>,
    laser_pos: Option<(f32, f32)>,
    frame_count: u64,
}

// GUI application using egui
struct BallDetectApp {
    state: Arc<Mutex<GuiState>>,
    texture: Option<egui::TextureHandle>,
}

impl eframe::App for BallDetectApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Read state quickly and release lock immediately to avoid deadlock
        // Clone only what we need while holding the lock briefly
        let (image_opt, ball_bbox, laser_pos, frame_count) = {
            let state = self.state.lock().unwrap();
            (
                state.image.clone(),
                state.ball_bbox,
                state.laser_pos,
                state.frame_count,
            )
        };
        
        // Do all expensive operations OUTSIDE the lock
        if let Some(img) = image_opt {
            // Convert RgbImage to marked image with bounding boxes
            let mut marked_img = img.clone();
            
            // Draw ball bounding box (red)
            if let Some((x1, y1, x2, y2)) = ball_bbox {
                draw_bbox(&mut marked_img, x1, y1, x2, y2, 3, Rgb([255, 0, 0]));
            }
            
            // Draw laser position (green circle)
            if let Some((lx, ly)) = laser_pos {
                let lx_u32 = lx as u32;
                let ly_u32 = ly as u32;
                let radius = 5u32;
                for dy in 0..=radius * 2 {
                    for dx in 0..=radius * 2 {
                        let x = lx_u32.saturating_add(dx).saturating_sub(radius);
                        let y = ly_u32.saturating_add(dy).saturating_sub(radius);
                        if x < marked_img.width() && y < marked_img.height() {
                            let dist_sq = ((dx as i32 - radius as i32).pow(2) + (dy as i32 - radius as i32).pow(2)) as u32;
                            if dist_sq <= radius.pow(2) {
                                marked_img.put_pixel(x, y, Rgb([0, 255, 0]));
                            }
                        }
                    }
                }
            }
            
            // Convert to egui color image
            let size = [marked_img.width() as usize, marked_img.height() as usize];
            let pixels: Vec<egui::Color32> = marked_img
                .pixels()
                .map(|p| egui::Color32::from_rgb(p[0], p[1], p[2]))
                .collect();
            
            let color_image = egui::ColorImage {
                size,
                pixels,
            };
            
            // Update texture
            self.texture = Some(ctx.load_texture("camera_image", color_image, egui::TextureOptions::LINEAR));
        }
        
        // Draw UI - use the values we already read (no need to lock again)
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Ball Detection - Real-time View");
            
            if let Some(ref texture) = self.texture {
                ui.label(format!("Frame: {}", frame_count));
                
                if let Some((x1, y1, x2, y2)) = ball_bbox {
                    ui.label(format!("Ball detected at: ({:.1}, {:.1}) to ({:.1}, {:.1})", x1, y1, x2, y2));
                } else {
                    ui.label("No ball detected");
                }
                
                if let Some((lx, ly)) = laser_pos {
                    ui.label(format!("Laser detected at: ({:.1}, {:.1})", lx, ly));
                } else {
                    ui.label("No laser detected");
                }
                
                // Display image
                ui.add(egui::Image::new(texture).fit_to_original_size(1.0));
            } else {
                ui.label("Waiting for image...");
            }
        });
        
        // Request repaint
        ctx.request_repaint();
    }
}

// Parse laser position string: "x,y" or "none"
fn parse_laser_position(s: &str) -> Result<(f32, f32), BallDetectError> {
    if s == "none" {
        return Err(BallDetectError::Detection("No laser detected".to_string()));
    }
    
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(BallDetectError::Detection(format!("Invalid laser position format: {}", s)));
    }
    
    let x = parts[0].parse::<f32>()
        .map_err(|e| BallDetectError::Detection(format!("Failed to parse laser x: {:?}", e)))?;
    let y = parts[1].parse::<f32>()
        .map_err(|e| BallDetectError::Detection(format!("Failed to parse laser y: {:?}", e)))?;
    
    Ok((x, y))
}

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

// YOLO-based laser detector using ONNX Runtime
// Similar to BallDetector but for detecting laser dot position
struct LaserDetector {
    session: ort::session::Session,
    input_shape: (usize, usize), // (height, width) for model input
    confidence_threshold: f32,
}

impl LaserDetector {
    fn new(model_path: &str, confidence_threshold: f32) -> Result<Self, BallDetectError> {
        let model_path = Path::new(model_path);
        if !model_path.exists() {
            return Err(BallDetectError::Ort(format!(
                "Laser model file not found: {}. Export your YOLO model to ONNX format first.",
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

    // Detect laser dot in image using YOLO
    // Input: PNG image data
    // Output: center position (x, y) or None
    fn detect(
        &mut self,
        png_data: &[u8],
    ) -> Result<Option<(f32, f32)>, BallDetectError> {
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

        // Find best laser detection (class 0)
        // YOLO output format is [batch, outputs, anchors] = [1, 5, 8400]
        // where outputs are [x_center, y_center, width, height, confidence]
        let mut best_detection: Option<(f32, f32)> = None;
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
                // Scale from model input size to original image size
                let scale_x = width as f32 / self.input_shape.1 as f32;
                let scale_y = height as f32 / self.input_shape.0 as f32;

                // Scale center coordinates to original image coordinates
                let x = center_x * scale_x;
                let y = center_y * scale_y;

                // Clamp to image boundaries
                let x = x.max(0.0).min(width as f32);
                let y = y.max(0.0).min(height as f32);

                best_detection = Some((x, y));
                best_confidence = class_confidence;
            }
        }

        Ok(best_detection)
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Shared state for GUI
    let gui_state = Arc::new(Mutex::new(GuiState {
        image: None,
        ball_bbox: None,
        laser_pos: None,
        frame_count: 0,
    }));
    
    // Spawn ROS processing task in background thread
    let gui_state_clone = gui_state.clone();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async_main(gui_state_clone))
            .unwrap_or_else(|e| eprintln!("ROS processing error: {:?}", e));
    });
    
    // Run GUI on main thread (required by winit)
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Ball Detection Viewer")
            .with_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    
    let app = BallDetectApp {
        state: gui_state,
        texture: None,
    };
    
    eframe::run_native(
        "Ball Detection Viewer",
        options,
        Box::new(|_cc| Ok(Box::new(app))),
    )?;
    
    Ok(())
}

async fn async_main(gui_state: Arc<Mutex<GuiState>>) -> Result<(), Box<dyn std::error::Error>> {
    println!("Ball Detection Node (YOLO-based with ONNX Runtime)");
    println!("====================================================");
    println!("This node runs on a more powerful device than the Raspberry Pi");
    println!("It subscribes to /image from the Pi and publishes bounding box coordinates");
    println!();

    // Get model paths from environment or use defaults
    let ball_model_path = std::env::var("YOLO_MODEL_PATH")
        .unwrap_or_else(|_| "./best.onnx".to_string());
    let laser_model_path = std::env::var("LASER_MODEL_PATH")
        .unwrap_or_else(|_| "./laser_best.onnx".to_string());
    let confidence_threshold = std::env::var("YOLO_CONFIDENCE_THRESHOLD")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.55);
    let laser_confidence_threshold = std::env::var("LASER_CONFIDENCE_THRESHOLD")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.55);

    println!("Loading ball detection YOLO ONNX model from: {}", ball_model_path);
    println!("Confidence threshold: {:.2}", confidence_threshold);
    println!();

    let mut ball_detector = BallDetector::new(&ball_model_path, confidence_threshold)?;
    println!("Ball detection model loaded successfully");
    println!();

    println!("Loading laser detection YOLO ONNX model from: {}", laser_model_path);
    println!("Laser confidence threshold: {:.2}", laser_confidence_threshold);
    println!();

    let mut laser_detector = LaserDetector::new(&laser_model_path, laser_confidence_threshold)?;
    println!("Laser detection model loaded successfully");
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

    // Create topic sender for ball coordinates
    println!("Creating publisher for ball coordinates...");
    let (tx_coords, _node_pub) = create_topic_sender::<StringMsg>(
        "ball_detect_node",
        "/ball_coords",
        QosProfile::default(),
    )
    .map_err(|e| BallDetectError::Ros2(format!("Failed to create topic sender: {:?}", e)))?;
    
    println!("Publisher created successfully");
    println!("  Topic: /ball_coords");
    println!("  Message type: std_msgs/String");
    println!("  Format: \"x1,y1,x2,y2\" or \"none\"");
    println!();

    // Create topic sender for laser position
    println!("Creating publisher for laser position...");
    let (tx_laser, _node_laser) = create_topic_sender::<StringMsg>(
        "ball_detect_node",
        "/laser_pos",
        QosProfile::default(),
    )
    .map_err(|e| BallDetectError::Ros2(format!("Failed to create laser topic sender: {:?}", e)))?;
    
    println!("Laser publisher created successfully");
    println!("  Topic: /laser_pos");
    println!("  Message type: std_msgs/String");
    println!("  Format: \"x,y\" or \"none\"");
    println!();

    // Sliding window to save last 3 images
    // Store: (frame_number, image_data)
    let image_buffer: Arc<Mutex<Vec<(u64, Vec<u8>)>>> = Arc::new(Mutex::new(Vec::new()));

    let mut frame_count = 0u64;
    let mut last_log_time = Instant::now();
    
    // Process incoming messages from channel
    while let Some(msg) = rx.recv().await {
        frame_count += 1;
        
        if frame_count == 1 {
            println!("✓ First message received!");
            println!("  Encoding: {}", msg.encoding);
            println!("  Size: {}x{}", msg.width, msg.height);
        }
        
        // Convert image data to PNG format for YOLO processing
        // Supports: rgb8 (RGB3/rgb24 raw), jpeg/mjpeg (compressed JPEG), png (already PNG)
        let png_data = if msg.encoding == "rgb8" {
            // Convert RGB3/rgb24 (24-bit RGB 8-8-8) raw data to PNG
            // ROS uses "rgb8" encoding name for RGB3 (V4L2) / rgb24 (ffmpeg) format
            match rgb8_to_png(&msg.data, msg.width, msg.height) {
                Ok(png) => png,
                Err(e) => {
                    eprintln!("Frame #{}: Failed to convert RGB3 to PNG: {}", frame_count, e);
                    continue;
                }
            }
        } else if msg.encoding == "jpeg" || msg.encoding == "mjpeg" {
            // Decode JPEG/MJPEG to image, then convert to PNG
            // The image crate can decode JPEG directly
            let img = match image::load_from_memory(&msg.data) {
                Ok(img) => img,
                Err(e) => {
                    eprintln!("Frame #{}: Failed to decode JPEG: {}", frame_count, e);
                    continue;
                }
            };
            
            // Convert to PNG bytes
            let mut png_bytes = Vec::new();
            {
                let mut cursor = std::io::Cursor::new(&mut png_bytes);
                if let Err(e) = img.write_to(&mut cursor, image::ImageFormat::Png) {
                    eprintln!("Frame #{}: Failed to encode PNG from JPEG: {}", frame_count, e);
                    continue;
                }
            }
            png_bytes
        } else if msg.encoding == "png" {
            // Already PNG, use directly
            msg.data.clone()
        } else {
            eprintln!("Frame #{}: Unsupported encoding '{}', expected 'rgb8' (RGB3/rgb24), 'jpeg'/'mjpeg' (MJPEG), or 'png'", frame_count, msg.encoding);
            continue;
        };
        
        // Save image to sliding window buffer (keep last 3 images)
        // unwrap is safe here: mutex poisoning indicates a programming error that should panic
        let old_frame_to_remove = {
            let mut buffer = image_buffer.lock().unwrap();
            buffer.push((frame_count, png_data.clone()));
            
            // Keep only last 3 images in memory
            let old_frame = if buffer.len() > 3 {
                let removed = buffer.remove(0);
                Some(removed.0)
            } else {
                None
            };
            
            // Save the new image to disk
            let filename = format!("frame_{:06}.png", frame_count);
            if let Err(e) = std::fs::write(&filename, &png_data) {
                eprintln!("Failed to save image {}: {:?}", filename, e);
            } else if frame_count <= 3 || frame_count % 30 == 0 {
                println!("Saved image: {} (keeping last 3 frames)", filename);
            }
            
            old_frame
        };
        
        // Clean up old image file if we removed one from buffer
        if let Some(old_frame) = old_frame_to_remove {
            let old_filename = format!("frame_{:06}.png", old_frame);
            if let Err(e) = std::fs::remove_file(&old_filename) {
                // Ignore errors (file might not exist)
                eprintln!("Note: Could not remove old image {}: {:?}", old_filename, e);
            }
        }
        
        // Load image for marking
        let rgb_img = match image::load_from_memory(&png_data) {
            Ok(img) => img.to_rgb8(),
            Err(e) => {
                eprintln!("Failed to load image: {:?}", e);
                continue;
            }
        };
        
        // Run ball detection on PNG data
        let (coords_string, bbox) = match ball_detector.detect(&png_data) {
            Ok(Some((x1, y1, x2, y2))) => {
                println!("Frame #{}: Ball detected at ({:.1}, {:.1}) to ({:.1}, {:.1})", 
                    frame_count, x1, y1, x2, y2);
                (format!("{:.2},{:.2},{:.2},{:.2}", x1, y1, x2, y2), Some((x1, y1, x2, y2)))
            }
            Ok(None) => {
                if frame_count % 30 == 0 {
                    println!("Frame #{}: No ball detected", frame_count);
                }
                ("none".to_string(), None)
            }
            Err(e) => {
                eprintln!("Ball detection error on frame #{}: {}", frame_count, e);
                ("none".to_string(), None)
            }
        };

        // Run laser detection on PNG data
        let laser_pos_string = match laser_detector.detect(&png_data) {
            Ok(Some((x, y))) => {
                if frame_count % 30 == 0 {
                    println!("Frame #{}: Laser detected at ({:.1}, {:.1})", frame_count, x, y);
                }
                format!("{:.2},{:.2}", x, y)
            }
            Ok(None) => {
                if frame_count % 30 == 0 {
                    println!("Frame #{}: No laser detected", frame_count);
                }
                "none".to_string()
            }
            Err(e) => {
                eprintln!("Laser detection error on frame #{}: {}", frame_count, e);
                "none".to_string()
            }
        };

        // Update GUI state
        {
            let mut state = gui_state.lock().unwrap();
            state.image = Some(rgb_img.clone());
            state.ball_bbox = bbox;
            state.laser_pos = if laser_pos_string != "none" {
                parse_laser_position(&laser_pos_string).ok()
            } else {
                None
            };
            state.frame_count = frame_count;
        }
        
        // Draw bounding boxes and save marked image
        let mut marked_img = rgb_img.clone();
        if let Some((x1, y1, x2, y2)) = bbox {
            // Draw red bounding box for ball
            draw_bbox(&mut marked_img, x1, y1, x2, y2, 3, Rgb([255, 0, 0]));
        }
        
        // Draw laser position if detected
        if laser_pos_string != "none" {
            if let Ok((lx, ly)) = parse_laser_position(&laser_pos_string) {
                // Draw a small circle/cross for laser position (green)
                let lx_u32 = lx as u32;
                let ly_u32 = ly as u32;
                let radius = 5u32;
                for dy in 0..=radius * 2 {
                    for dx in 0..=radius * 2 {
                        let x = lx_u32.saturating_add(dx).saturating_sub(radius);
                        let y = ly_u32.saturating_add(dy).saturating_sub(radius);
                        if x < marked_img.width() && y < marked_img.height() {
                            let dist_sq = ((dx as i32 - radius as i32).pow(2) + (dy as i32 - radius as i32).pow(2)) as u32;
                            if dist_sq <= radius.pow(2) {
                                marked_img.put_pixel(x, y, Rgb([0, 255, 0]));
                            }
                        }
                    }
                }
            }
        }
        
        // Save marked image as image.png
        if let Err(e) = marked_img.save("image.png") {
            eprintln!("Failed to save marked image: {:?}", e);
        } else if frame_count % 30 == 0 {
            println!("Saved marked image to image.png");
        }
        
        // Publish ball coordinates as string to /ball_coords topic
        let coord_msg = StringMsg {
            data: coords_string.clone(),
        };
        
        match tx_coords.send(coord_msg).await {
            Ok(_) => {
                // Log periodically to confirm coordinates are being sent
                if frame_count % 30 == 0 {
                    println!("Published ball coordinates to /ball_coords: {}", coords_string);
                }
            }
            Err(e) => {
                eprintln!("Failed to send ball coordinates to /ball_coords: {:?}", e);
                eprintln!("  This may indicate a connection issue with subscribers");
            }
        }

        // Publish laser position as string to /laser_pos topic
        let laser_msg = StringMsg {
            data: laser_pos_string.clone(),
        };
        
        match tx_laser.send(laser_msg).await {
            Ok(_) => {
                // Log periodically to confirm laser position is being sent
                if frame_count % 30 == 0 {
                    println!("Published laser position to /laser_pos: {}", laser_pos_string);
                }
            }
            Err(e) => {
                eprintln!("Failed to send laser position to /laser_pos: {:?}", e);
                eprintln!("  This may indicate a connection issue with subscribers");
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
