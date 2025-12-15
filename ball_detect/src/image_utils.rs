// Utility module for image format conversion
// Converts RGB3/rgb24 (24-bit RGB 8-8-8) raw data to PNG format for YOLO processing
// RGB3 (V4L2) / rgb24 (ffmpeg): Stepwise 16x16 - 16376x16376 with step 1/1

use image::{ImageBuffer, Rgb, RgbImage};
use std::io::Cursor;

// Draw a bounding box rectangle on an RGB image
// x1, y1: top-left corner, x2, y2: bottom-right corner
// line_width: thickness of the rectangle border in pixels
// color: RGB color for the rectangle (default: red)
pub fn draw_bbox(
    img: &mut RgbImage,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    line_width: u32,
    color: Rgb<u8>,
) {
    let width = img.width();
    let height = img.height();
    
    // Clamp coordinates to image bounds
    let x1 = x1.max(0.0).min(width as f32) as u32;
    let y1 = y1.max(0.0).min(height as f32) as u32;
    let x2 = x2.max(0.0).min(width as f32) as u32;
    let y2 = y2.max(0.0).min(height as f32) as u32;
    
    // Draw top and bottom horizontal lines
    for y in y1..(y1 + line_width).min(height) {
        for x in x1..x2.min(width) {
            img.put_pixel(x, y, color);
        }
    }
    for y in (y2.saturating_sub(line_width))..y2.min(height) {
        for x in x1..x2.min(width) {
            img.put_pixel(x, y, color);
        }
    }
    
    // Draw left and right vertical lines
    for x in x1..(x1 + line_width).min(width) {
        for y in y1..y2.min(height) {
            img.put_pixel(x, y, color);
        }
    }
    for x in (x2.saturating_sub(line_width))..x2.min(width) {
        for y in y1..y2.min(height) {
            img.put_pixel(x, y, color);
        }
    }
}

// Error type for image conversion
#[derive(Debug)]
pub enum ImageUtilsError {
    InvalidSize(String),
    Encoding(String),
}

impl std::fmt::Display for ImageUtilsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ImageUtilsError::InvalidSize(msg) => write!(f, "Invalid size: {}", msg),
            ImageUtilsError::Encoding(msg) => write!(f, "Encoding error: {}", msg),
        }
    }
}

impl std::error::Error for ImageUtilsError {}

// Convert RGB3/rgb24 (24-bit RGB 8-8-8) raw data to PNG bytes
// RGB3 (V4L2) / rgb24 (ffmpeg) format: 3 bytes per pixel (R, G, B) in row-major order
// Input: raw RGB3/rgb24 data (received as "rgb8" encoding in ROS), width, height
// Output: PNG-encoded bytes
pub fn rgb8_to_png(rgb_data: &[u8], width: u32, height: u32) -> Result<Vec<u8>, ImageUtilsError> {
    // Verify RGB data size
    let expected_size = (width * height * 3) as usize;
    if rgb_data.len() != expected_size {
        return Err(ImageUtilsError::InvalidSize(format!(
            "RGB data size mismatch: expected {} bytes ({}x{}x3), got {} bytes",
            expected_size, width, height, rgb_data.len()
        )));
    }
    
    // Create RGB image from raw data
    // ImageBuffer expects data in row-major order: [R, G, B, R, G, B, ...]
    let img: RgbImage = ImageBuffer::<Rgb<u8>, _>::from_raw(width, height, rgb_data.to_vec())
        .ok_or_else(|| ImageUtilsError::InvalidSize(format!(
            "Failed to create image buffer from {} bytes (expected {}x{}x3 = {} bytes)",
            rgb_data.len(), width, height, expected_size
        )))?;
    
    // Encode to PNG using write_to method
    let mut png_bytes = Vec::new();
    {
        let mut cursor = Cursor::new(&mut png_bytes);
        img.write_to(&mut cursor, image::ImageFormat::Png)
            .map_err(|e| ImageUtilsError::Encoding(format!("Failed to encode PNG: {:?}", e)))?;
    }
    
    Ok(png_bytes)
}

