use std::env;
use std::fs;
use std::path::Path;

fn main() {
    // Tell Cargo to rerun this build script if config.toml changes
    println!("cargo:rerun-if-changed=config.toml");
    
    // Read config.toml
    let config_path = Path::new("config.toml");
    let config_content = fs::read_to_string(config_path)
        .expect("Failed to read config.toml");
    
    // Parse TOML manually (simple enough for our needs)
    // Look for [camera] section and parse key = value pairs
    let mut in_camera_section = false;
    let mut device_path = String::from("/dev/video0");
    let mut width = 512u32;
    let mut height = 512u32;
    let mut buffers = 4u32;
    let mut publish_every_n = 15u64;
    let mut target_fps = 10u32;
    
    for line in config_content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        
        // Check for section headers
        if line.starts_with('[') && line.ends_with(']') {
            in_camera_section = line == "[camera]";
            continue;
        }
        
        // Only parse if we're in the [camera] section
        if !in_camera_section {
            continue;
        }
        
        // Parse key = value pairs
        if let Some((key, value)) = line.split_once('=') {
            let key = key.trim();
            let value = value.trim().trim_matches('"'); // Remove quotes if present
            
            match key {
                "device_path" => {
                    device_path = value.to_string();
                }
                "width" => {
                    if let Ok(v) = value.parse::<u32>() {
                        width = v;
                    }
                }
                "height" => {
                    if let Ok(v) = value.parse::<u32>() {
                        height = v;
                    }
                }
                "buffers" => {
                    if let Ok(v) = value.parse::<u32>() {
                        buffers = v;
                    }
                }
                "publish_every_n" => {
                    if let Ok(v) = value.parse::<u64>() {
                        publish_every_n = v;
                    }
                }
                "target_fps" => {
                    if let Ok(v) = value.parse::<u32>() {
                        target_fps = v;
                    }
                }
                _ => {}
            }
        }
    }
    
    // Generate constants in a file that main.rs can include
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("config.rs");
    
    fs::write(
        &dest_path,
        format!(
            r#"// Auto-generated from config.toml at build time
pub const CAM_DEVICE_PATH: &str = "{}";
pub const CAM_WIDTH: u32 = {};
pub const CAM_HEIGHT: u32 = {};
pub const CAM_BUFFERS: u32 = {};
pub const PUBLISH_EVERY_N: u64 = {};
pub const TARGET_FPS: u32 = {};
"#,
            device_path, width, height, buffers, publish_every_n, target_fps
        ),
    )
    .expect("Failed to write config.rs");
}

