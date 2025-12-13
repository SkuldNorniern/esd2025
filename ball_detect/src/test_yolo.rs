// Simple YOLO detection test - reads input.png and outputs detection positions
// Based on: https://medium.com/@alfred.weirich/rust-ort-onnx-real-time-yolo-on-a-live-webcam-part-1-b6edfb50bf9b
use std::path::Path;
use image;
use ndarray::Array4;
use ort::{
    session::builder::SessionBuilder,
    value::Value,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("YOLO Detection Test");
    println!("===================");
    
    // Load model
    let model_path = std::env::var("YOLO_MODEL_PATH")
        .unwrap_or_else(|_| "./best.onnx".to_string());
    
    println!("Loading model from: {}", model_path);
    
    // Initialize ONNX Runtime
    // Environment is created automatically when needed
    let mut session = SessionBuilder::new()?
        .commit_from_file(model_path)?;
    
    println!("Model loaded successfully");
    
    // Load input image
    let input_path = "input.png";
    if !Path::new(input_path).exists() {
        return Err(format!("Input image not found: {}", input_path).into());
    }
    
    println!("Loading image: {}", input_path);
    let img = image::open(input_path)?;
    let rgb_img = img.to_rgb8();
    let (width, height) = rgb_img.dimensions();
    println!("Image size: {}x{}", width, height);
    
    // Resize to model input size (640x640)
    let input_size = 640;
    let resized = img.resize_exact(
        input_size,
        input_size,
        image::imageops::FilterType::Triangle,
    );
    
    // Convert to normalized tensor [1, 3, 640, 640]
    let input_size_usize = input_size as usize;
    let mut input_array = Array4::<f32>::zeros((1, 3, input_size_usize, input_size_usize));
    
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
    
    println!("Running inference...");
    
    // Run inference
    let input_tensor = Value::from_array(input_array.into_dyn())?;
    let inputs = ort::inputs!["images" => input_tensor];
    let outputs = session.run(inputs)?;
    
    // Extract output - returns (shape, data) tuple
    let (output_shape, output_data) = outputs["output0"]
        .try_extract_tensor::<f32>()?;
    
    println!("Output shape: {:?}", output_shape);
    
    // Parse detections
    // Shape implements IntoIterator, collect dimensions
    let shape_dims: Vec<usize> = output_shape.iter().map(|&d| d as usize).collect();
    if shape_dims.len() != 3 {
        return Err(format!("Unexpected output shape: {:?}", shape_dims).into());
    }
    
    let batch_size = shape_dims[0];
    let dim1 = shape_dims[1];
    let dim2 = shape_dims[2];
    
    // Try both possible layouts: [batch, outputs, anchors] or [batch, anchors, outputs]
    // Shape is [1, 5, 8400] - could be either format
    let (num_outputs, num_anchors) = if dim1 == 5 {
        // Likely [batch, outputs=5, anchors=8400]
        (dim1, dim2)
    } else {
        // Likely [batch, anchors=8400, outputs=5]
        (dim2, dim1)
    };
    
    println!("Batch: {}, Dim1: {}, Dim2: {}", batch_size, dim1, dim2);
    println!("Interpreting as: Outputs: {}, Anchors: {}", num_outputs, num_anchors);
    
    if batch_size != 1 || num_outputs != 5 {
        return Err(format!("Invalid output shape: {:?}", shape_dims).into());
    }
    
    // YOLO output format is [batch, outputs, anchors] = [1, 5, 8400]
    // where outputs are [x_center, y_center, width, height, confidence]
    // Coordinates are already in pixel space for the model input size (640x640)
    
    let confidence_threshold = 0.25;
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
        
        if class_confidence >= confidence_threshold && class_confidence > best_confidence {
            // Coordinates are already in pixel space for the model input (640x640)
            // No need to multiply by input_size
            let scale_x = width as f32 / input_size as f32;
            let scale_y = height as f32 / input_size as f32;
            
            // Use coordinates directly (they're already in pixels for 640x640)
            let abs_center_x = center_x;
            let abs_center_y = center_y;
            let abs_width = bbox_width;
            let abs_height = bbox_height;
            
            // Convert center+size to x1,y1,x2,y2 format
            let x1_model = abs_center_x - abs_width / 2.0;
            let y1_model = abs_center_y - abs_height / 2.0;
            let x2_model = abs_center_x + abs_width / 2.0;
            let y2_model = abs_center_y + abs_height / 2.0;
            
            // Scale from model input size (640x640) to original image size
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
    
    // Print results
    println!("\nDetection Results:");
    println!("==================");
    if let Some((x1, y1, x2, y2)) = best_detection {
        println!("Ball detected!");
        println!("  Confidence: {:.2}%", best_confidence * 100.0);
        println!("  Bounding box: ({:.1}, {:.1}) to ({:.1}, {:.1})", x1, y1, x2, y2);
        println!("  Center: ({:.1}, {:.1})", (x1 + x2) / 2.0, (y1 + y2) / 2.0);
        println!("  Size: {:.1} x {:.1} pixels", x2 - x1, y2 - y1);
    } else {
        println!("No ball detected (confidence threshold: {:.2})", confidence_threshold);
    }
    
    Ok(())
}
