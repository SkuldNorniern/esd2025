use ros2_client::{Context, Node, NodeOptions, QosProfile};
use std::time::Duration;

// Error type for ball detection operations
#[derive(Debug)]
enum BallDetectError {
    Ros2(String),
    Image(String),
    Detection(String),
    OpenCv(String),
}

impl std::fmt::Display for BallDetectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BallDetectError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            BallDetectError::Image(msg) => write!(f, "Image error: {}", msg),
            BallDetectError::Detection(msg) => write!(f, "Detection error: {}", msg),
            BallDetectError::OpenCv(msg) => write!(f, "OpenCV error: {}", msg),
        }
    }
}

impl std::error::Error for BallDetectError {}

// OpenCV-based ball detector for offloaded processing
// This runs on a more powerful device than the Raspberry Pi
struct BallDetector {
    // HSV thresholds for ball color (tunable via environment or config)
    h_min: u8,
    h_max: u8,
    s_min: u8,
    s_max: u8,
    v_min: u8,
    v_max: u8,
    // Minimum blob area to consider (filters noise)
    min_area: f64,
    // Maximum blob area (filters large false positives)
    max_area: f64,
}

impl BallDetector {
    fn new() -> Result<Self, BallDetectError> {
        // Initialize OpenCV
        opencv::core::set_num_threads(4)
            .map_err(|e| BallDetectError::OpenCv(format!("Failed to set OpenCV threads: {:?}", e)))?;

        // Default HSV thresholds (adjust based on your ball color)
        // These can be overridden via environment variables or config file
        let h_min = std::env::var("BALL_H_MIN")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        let h_max = std::env::var("BALL_H_MAX")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(179);
        let s_min = std::env::var("BALL_S_MIN")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(50);
        let s_max = std::env::var("BALL_S_MAX")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(255);
        let v_min = std::env::var("BALL_V_MIN")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(50);
        let v_max = std::env::var("BALL_V_MAX")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(255);

        Ok(Self {
            h_min,
            h_max,
            s_min,
            s_max,
            v_min,
            v_max,
            min_area: 50.0,  // Minimum blob area in pixels
            max_area: 50000.0, // Maximum blob area in pixels
        })
    }

    // Detect ball in image using OpenCV
    // Input: image data, width, height, encoding
    // Output: (center_x, center_y, radius, confidence) or None
    fn detect(&self, image_data: &[u8], width: u32, height: u32, encoding: &str) -> Result<Option<(f32, f32, f32, f32)>, BallDetectError> {
        // Convert image data to OpenCV Mat
        let mat = match encoding {
            "rgb8" => {
                // Create Mat from RGB data
                unsafe {
                    opencv::core::Mat::new_rows_cols_with_data(
                        height as i32,
                        width as i32,
                        opencv::core::CV_8UC3,
                        image_data.as_ptr() as *mut std::ffi::c_void,
                        opencv::core::Mat_AUTO_STEP,
                    )
                    .map_err(|e| BallDetectError::OpenCv(format!("Failed to create Mat: {:?}", e)))?
                }
            }
            "bgr8" => {
                // BGR is OpenCV's native format
                unsafe {
                    opencv::core::Mat::new_rows_cols_with_data(
                        height as i32,
                        width as i32,
                        opencv::core::CV_8UC3,
                        image_data.as_ptr() as *mut std::ffi::c_void,
                        opencv::core::Mat_AUTO_STEP,
                    )
                    .map_err(|e| BallDetectError::OpenCv(format!("Failed to create Mat: {:?}", e)))?
                }
            }
            _ => {
                return Err(BallDetectError::Image(format!("Unsupported encoding: {}", encoding)));
            }
        };

        // Convert BGR to HSV
        let mut hsv = opencv::core::Mat::default();
        opencv::imgproc::cvt_color(&mat, &mut hsv, opencv::imgproc::COLOR_BGR2HSV, 0)
            .map_err(|e| BallDetectError::OpenCv(format!("Failed to convert to HSV: {:?}", e)))?;

        // Create HSV threshold range
        let lower_bound = opencv::core::Scalar::new(
            self.h_min as f64,
            self.s_min as f64,
            self.v_min as f64,
            0.0,
        );
        let upper_bound = opencv::core::Scalar::new(
            self.h_max as f64,
            self.s_max as f64,
            self.v_max as f64,
            0.0,
        );

        // Apply color threshold
        let mut mask = opencv::core::Mat::default();
        opencv::core::in_range(&hsv, &lower_bound, &upper_bound, &mut mask)
            .map_err(|e| BallDetectError::OpenCv(format!("Failed to threshold: {:?}", e)))?;

        // Apply morphological operations to reduce noise
        let kernel = opencv::imgproc::get_structuring_element(
            opencv::imgproc::MORPH_ELLIPSE,
            opencv::core::Size::new(5, 5),
            opencv::core::Point::new(-1, -1),
        )
        .map_err(|e| BallDetectError::OpenCv(format!("Failed to create kernel: {:?}", e)))?;

        let mut mask_cleaned = opencv::core::Mat::default();
        opencv::imgproc::morphology_ex(
            &mask,
            &mut mask_cleaned,
            opencv::imgproc::MORPH_OPEN,
            &kernel,
            opencv::core::Point::new(-1, -1),
            2,
            opencv::core::BORDER_CONSTANT,
            opencv::imgproc::morphology_default_border_value().unwrap(),
        )
        .map_err(|e| BallDetectError::OpenCv(format!("Failed morphology: {:?}", e)))?;

        // Find contours
        let mut contours = opencv::core::Vector::<opencv::core::Vector<opencv::core::Point>>::new();
        let mut hierarchy = opencv::core::Mat::default();
        opencv::imgproc::find_contours(
            &mask_cleaned,
            &mut contours,
            &mut hierarchy,
            opencv::imgproc::RETR_EXTERNAL,
            opencv::imgproc::CHAIN_APPROX_SIMPLE,
            opencv::core::Point::new(0, 0),
        )
        .map_err(|e| BallDetectError::OpenCv(format!("Failed to find contours: {:?}", e)))?;

        if contours.len() == 0 {
            return Ok(None);
        }

        // Find largest contour that meets size criteria
        let mut best_contour: Option<opencv::core::Vector<opencv::core::Point>> = None;
        let mut best_area = 0.0;

        for i in 0..contours.len() {
            let contour = contours.get(i)
                .map_err(|e| BallDetectError::OpenCv(format!("Failed to get contour: {:?}", e)))?;
            let area = opencv::imgproc::contour_area(&contour, false)
                .map_err(|e| BallDetectError::OpenCv(format!("Failed to get area: {:?}", e)))?;

            if area >= self.min_area && area <= self.max_area && area > best_area {
                best_area = area;
                best_contour = Some(contour);
            }
        }

        let contour = match best_contour {
            Some(c) => c,
            None => return Ok(None),
        };

        // Fit circle to contour
        let mut center = opencv::core::Point2f::default();
        let mut radius = 0f32;
        opencv::imgproc::min_enclosing_circle(&contour, &mut center, &mut radius)
            .map_err(|e| BallDetectError::OpenCv(format!("Failed to fit circle: {:?}", e)))?;

        // Calculate confidence based on circularity
        let perimeter = opencv::imgproc::arc_length(&contour, true)
            .map_err(|e| BallDetectError::OpenCv(format!("Failed to get perimeter: {:?}", e)))?;
        let circularity = if perimeter > 0.0 {
            4.0 * std::f32::consts::PI * best_area as f32 / (perimeter * perimeter)
        } else {
            0.0
        };

        // Confidence combines circularity and size
        let size_confidence = ((best_area / self.max_area).min(1.0) * 0.3) as f32;
        let confidence = (circularity * 0.7 + size_confidence).min(1.0);

        // Only return if confidence is reasonable
        if confidence < 0.3 {
            return Ok(None);
        }

        Ok(Some((center.x, center.y, radius, confidence)))
    }
}

fn main() -> Result<(), BallDetectError> {
    println!("Ball Detection Node (Offloaded Processing)");
    println!("===========================================");
    println!("This node runs on a more powerful device than the Raspberry Pi");
    println!("It subscribes to /image from the Pi and publishes detection results");
    println!();

    // Initialize ROS2 context
    // For network communication, ensure ROS2 DDS is configured correctly
    // Set ROS_DOMAIN_ID environment variable if needed (default is 0)
    let ctx = Context::new()
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create ROS2 context: {:?}", e)))?;
    
    let node = Node::new(&ctx, "ball_detect_node", &NodeOptions::new().enable_rosout(true))
        .map_err(|e| BallDetectError::Ros2(format!("Failed to create ROS2 node: {:?}", e)))?;

    println!("ROS2 node created: ball_detect_node");
    println!("ROS_DOMAIN_ID: {:?}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));

    println!();
    println!("Subscribing to /image topic...");
    // TODO: Create subscriber for /image topic
    // let subscriber = node.create_subscriber::<sensor_msgs::msg::Image>(
    //     "/image",
    //     &QosProfile::default(),
    // )
    // .map_err(|e| BallDetectError::Ros2(format!("Failed to create subscriber: {:?}", e)))?;

    println!("Creating publishers for ball detection results...");
    // TODO: Create publishers for ball position
    // Using Option A from README: separate topics
    // - /ball_center (geometry_msgs/PointStamped)
    // - /ball_radius (std_msgs/Float32)
    // - /ball_confidence (std_msgs/Float32)
    //
    // let center_pub = node.create_publisher::<geometry_msgs::msg::PointStamped>(
    //     "/ball_center",
    //     &QosProfile::default(),
    // )
    // .map_err(|e| BallDetectError::Ros2(format!("Failed to create center publisher: {:?}", e)))?;
    //
    // let radius_pub = node.create_publisher::<std_msgs::msg::Float32>(
    //     "/ball_radius",
    //     &QosProfile::default(),
    // )
    // .map_err(|e| BallDetectError::Ros2(format!("Failed to create radius publisher: {:?}", e)))?;
    //
    // let confidence_pub = node.create_publisher::<std_msgs::msg::Float32>(
    //     "/ball_confidence",
    //     &QosProfile::default(),
    // )
    // .map_err(|e| BallDetectError::Ros2(format!("Failed to create confidence publisher: {:?}", e)))?;

    let detector = BallDetector::new()?;
    println!("Ball detector initialized");
    println!("HSV thresholds: H[{}-{}] S[{}-{}] V[{}-{}]",
        detector.h_min, detector.h_max,
        detector.s_min, detector.s_max,
        detector.v_min, detector.v_max);
    println!("(Override via environment variables: BALL_H_MIN, BALL_H_MAX, etc.)");
    println!();

    println!("Waiting for images on /image topic...");
    println!("(Press Ctrl+C to stop)");
    println!();

    // Main processing loop
    // TODO: Receive images from ROS2 subscriber
    // For each image:
    //   1. Extract image data, width, height, encoding
    //   2. Run detector.detect()
    //   3. If ball found, publish results to /ball_center, /ball_radius, /ball_confidence
    //   4. If no ball, publish zero/empty values

    let mut frame_count = 0u64;
    loop {
        // This will be replaced with actual ROS2 message handling
        // Example structure:
        // match subscriber.receive(Duration::from_millis(100)) {
        //     Ok(Some(image_msg)) => {
        //         frame_count += 1;
        //         let image_data = image_msg.data.as_slice();
        //         let width = image_msg.width;
        //         let height = image_msg.height;
        //         let encoding = &image_msg.encoding;
        //
        //         match detector.detect(image_data, width, height, encoding)? {
        //             Some((x, y, radius, confidence)) => {
        //                 println!("Frame {}: Ball detected at ({:.1}, {:.1}), radius: {:.1}, confidence: {:.2}",
        //                     frame_count, x, y, radius, confidence);
        //
        //                 // Publish results
        //                 // ... create and publish messages
        //             }
        //             None => {
        //                 if frame_count % 30 == 0 {
        //                     println!("Frame {}: No ball detected", frame_count);
        //                 }
        //                 // Publish empty/zero values
        //             }
        //         }
        //     }
        //     Ok(None) => {
        //         // Timeout, continue
        //     }
        //     Err(e) => {
        //         eprintln!("Error receiving message: {:?}", e);
        //     }
        // }

        std::thread::sleep(Duration::from_millis(100));
    }
}
