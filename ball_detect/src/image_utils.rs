// Utility module for image format conversion
// Converts RGB3/rgb24 (24-bit RGB 8-8-8) raw data to PNG format for YOLO processing
// RGB3 (V4L2) / rgb24 (ffmpeg): Stepwise 16x16 - 16376x16376 with step 1/1

use image::{ImageBuffer, Rgb, RgbImage};
use std::io::Cursor;

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
