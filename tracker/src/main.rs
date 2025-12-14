use ros_wrapper::{create_topic_receiver, QosProfile, std_msgs::msg::String as StringMsg};
use software_pwm::SoftwarePwmServo;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// Error type for tracker operations
#[derive(Debug)]
enum TrackerError {
    Ros2(String),
    Gpio(String),
    Pwm(String),
    Control(String),
}

impl std::fmt::Display for TrackerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrackerError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            TrackerError::Gpio(msg) => write!(f, "GPIO error: {}", msg),
            TrackerError::Pwm(msg) => write!(f, "PWM error: {}", msg),
            TrackerError::Control(msg) => write!(f, "Control error: {}", msg),
        }
    }
}

impl std::error::Error for TrackerError {}

// Servo control using software PWM
// GPIO18: Pan (Left-Right)
// GPIO19: Tilt (Up-Down)
struct ServoController {
    pan_servo: SoftwarePwmServo,
    tilt_servo: SoftwarePwmServo,
}

impl ServoController {
    fn new() -> Result<Self, TrackerError> {
        // Initialize both servos using software PWM library
        // GPIO18: Pan (Left-Right)
        let pan_servo = SoftwarePwmServo::new(18)
            .map_err(|e| TrackerError::Pwm(format!("Failed to initialize pan servo on GPIO18: {:?}", e)))?;
        
        // GPIO19: Tilt (Up-Down)
        let tilt_servo = SoftwarePwmServo::new(19)
            .map_err(|e| TrackerError::Pwm(format!("Failed to initialize tilt servo on GPIO19: {:?}", e)))?;

        Ok(Self {
            pan_servo,
            tilt_servo,
        })
    }

    // Set pan servo position (0-180 degrees)
    fn set_pan(&self, angle: f64) -> Result<(), TrackerError> {
        let angle = angle.max(0.0).min(180.0);
        let angle_u8 = angle as u8;
        self.pan_servo.set_angle(angle_u8)
            .map_err(|e| TrackerError::Pwm(format!("Failed to set pan servo angle: {:?}", e)))?;
        Ok(())
    }

    // Set tilt servo position (0-180 degrees)
    fn set_tilt(&self, angle: f64) -> Result<(), TrackerError> {
        let angle = angle.max(0.0).min(180.0);
        let angle_u8 = angle as u8;
        self.tilt_servo.set_angle(angle_u8)
            .map_err(|e| TrackerError::Pwm(format!("Failed to set tilt servo angle: {:?}", e)))?;
        Ok(())
    }

    fn shutdown(&mut self) {
        // Set servos to center position before shutdown
        // SoftwarePwmServo will clean up automatically on drop
        self.set_pan(90.0).ok();
        self.set_tilt(90.0).ok();
    }
}

impl Drop for ServoController {
    fn drop(&mut self) {
        self.shutdown();
    }
}

// PID controller (starting with P-only as per README)
struct Controller {
    // Proportional gains
    kp_pan: f64,
    kp_tilt: f64,
    // Current servo positions (degrees)
    pan_angle: f64,
    tilt_angle: f64,
    // Image dimensions (for coordinate conversion)
    image_width: u32,
    image_height: u32,
    // Servo limits (degrees)
    pan_min: f64,
    pan_max: f64,
    tilt_min: f64,
    tilt_max: f64,
    // Rate limiting (max change per update)
    max_rate: f64,
    last_update: Instant,
    // Direction inversion flags (if servos are mounted backwards)
    invert_pan: bool,
    invert_tilt: bool,
}

impl Controller {
    fn new(image_width: u32, image_height: u32) -> Self {
        Self {
            kp_pan: 0.15,  // Higher gain for accurate tracking
            kp_tilt: 0.15,
            pan_angle: 90.0,  // Start at center
            tilt_angle: 90.0,
            image_width,
            image_height,
            pan_min: 0.0,
            pan_max: 180.0,
            tilt_min: 0.0,
            tilt_max: 180.0,
            max_rate: 10.0,  // Higher rate for faster response
            last_update: Instant::now(),
            // Set to true if servos are mounted in reverse
            // If tracker drifts right when ball is left, set invert_pan: true
            invert_pan: true,  // Try inverting if drifting wrong direction
            invert_tilt: false,
        }
    }

    // Update servo positions based on ball detection
    // Input: bounding box (x1, y1, x2, y2) in image coordinates
    fn update(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) -> (f64, f64) {
        // Calculate ball center
        let ball_center_x = (x1 + x2) / 2.0;
        let ball_center_y = (y1 + y2) / 2.0;

        // Calculate image center
        let image_center_x = self.image_width as f32 / 2.0;
        let image_center_y = self.image_height as f32 / 2.0;

        // Calculate error (pixels from center)
        let error_x = ball_center_x - image_center_x;
        let error_y = ball_center_y - image_center_y;

        // Normalize error to -1.0 to 1.0 range
        let normalized_error_x = error_x / image_center_x;
        let normalized_error_y = error_y / image_center_y;

        // Apply P-control - focus on accuracy, not smoothness
        // Image coordinates: (0,0) is top-left, x increases right, y increases down
        // Servo convention: 0° = left/up, 90° = center, 180° = right/down
        // 
        // If ball is to the RIGHT of center (positive error_x):
        //   - We need to pan RIGHT to follow it
        //   - Pan RIGHT means INCREASE angle (toward 180°)
        //   - So delta_pan should be POSITIVE when error_x is POSITIVE
        //
        // If ball is BELOW center (positive error_y):
        //   - We need to tilt DOWN to follow it
        //   - Tilt DOWN means INCREASE angle (toward 180°)
        //   - So delta_tilt should be POSITIVE when error_y is POSITIVE
        let mut delta_pan = self.kp_pan * normalized_error_x as f64 * 90.0; // Scale to degrees
        let mut delta_tilt = self.kp_tilt * normalized_error_y as f64 * 90.0;
        
        // Apply direction inversion if servos are mounted backwards
        if self.invert_pan {
            delta_pan = -delta_pan;
        }
        if self.invert_tilt {
            delta_tilt = -delta_tilt;
        }

        // Rate limiting
        let elapsed = self.last_update.elapsed().as_secs_f64();
        let max_delta = self.max_rate * elapsed.max(0.01); // Cap at reasonable rate
        
        let delta_pan = delta_pan.max(-max_delta).min(max_delta);
        let delta_tilt = delta_tilt.max(-max_delta).min(max_delta);

        // Update angles
        self.pan_angle += delta_pan;
        self.tilt_angle += delta_tilt;

        // Clamp to limits
        self.pan_angle = self.pan_angle.max(self.pan_min).min(self.pan_max);
        self.tilt_angle = self.tilt_angle.max(self.tilt_min).min(self.tilt_max);

        self.last_update = Instant::now();

        (self.pan_angle, self.tilt_angle)
    }

    // Reset to center position
    fn reset_to_center(&mut self) {
        self.pan_angle = 90.0;
        self.tilt_angle = 90.0;
    }
}

// Ball detection coordinates (latest values)
struct BallCoordinates {
    coords: Arc<Mutex<Option<(f32, f32, f32, f32)>>>,
    last_update: Arc<Mutex<Instant>>,
}

impl BallCoordinates {
    fn new() -> Self {
        Self {
            coords: Arc::new(Mutex::new(None)),
            last_update: Arc::new(Mutex::new(Instant::now())),
        }
    }

    fn get_coordinates(&self) -> Option<(f32, f32, f32, f32)> {
        *self.coords.lock().unwrap()
    }

    fn set_coordinates(&self, coords: Option<(f32, f32, f32, f32)>) {
        *self.coords.lock().unwrap() = coords;
        *self.last_update.lock().unwrap() = Instant::now();
    }

    fn is_stale(&self, timeout: Duration) -> bool {
        self.last_update.lock().unwrap().elapsed() > timeout
    }
}

// Parse coordinates string: "x1,y1,x2,y2" or "none"
fn parse_coordinates(s: &str) -> Option<(f32, f32, f32, f32)> {
    if s == "none" {
        return None;
    }
    
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 4 {
        return None;
    }
    
    let x1 = parts[0].parse().ok()?;
    let y1 = parts[1].parse().ok()?;
    let x2 = parts[2].parse().ok()?;
    let y2 = parts[3].parse().ok()?;
    
    Some((x1, y1, x2, y2))
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing tracker node...");
    println!("GPIO18: Pan servo (Left-Right)");
    println!("GPIO19: Tilt servo (Up-Down)");
    println!();
    println!("Note: This code requires Raspberry Pi hardware.");
    println!("It will not compile on macOS/Windows due to rppal dependency.");
    println!();

    // Initialize servo controller
    let servo = ServoController::new()?;
    println!("Servo controller initialized");

    // Set initial position to center
    servo.set_pan(90.0)?;
    servo.set_tilt(90.0)?;
    println!("Servos set to center position (90 degrees)");
    println!();

    // Create subscriber for ball detection coordinates
    println!("Subscribing to ball detection topic...");
    let coords = BallCoordinates::new();
    
    // Subscribe to /ball_coords topic (single topic with string format: "x1,y1,x2,y2" or "none")
    let (mut rx_coords, _node_coords) = create_topic_receiver::<StringMsg>(
        "tracker_node",
        "/ball_coords",
        QosProfile::default(),
    )?;

    println!("Subscriber created successfully");
    println!("  Topic: /ball_coords");
    println!("  Message type: std_msgs/String");
    println!("  Format: \"x1,y1,x2,y2\" or \"none\"");
    println!("ROS_DOMAIN_ID: {}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!();

    // Wait for subscribers to establish connections
    println!("Waiting for subscribers to establish connections...");
    println!("  (DDS discovery can take 5-10 seconds for cross-device communication)");
    for _i in 1..=10 {
        tokio::time::sleep(Duration::from_secs(1)).await;
        print!(".");
        std::io::Write::flush(&mut std::io::stdout())?;
    }
    println!();
    println!("Subscribers ready, waiting for ball detection data...");
    println!("(Press Ctrl+C to stop)");
    println!();

    // Image dimensions (should match camera resolution: 640x640)
    let image_width = 640;
    let image_height = 640;
    let mut controller = Controller::new(image_width, image_height);

    // Spawn task to receive coordinates from single topic
    let coords_clone = BallCoordinates {
        coords: Arc::clone(&coords.coords),
        last_update: Arc::clone(&coords.last_update),
    };

    tokio::spawn(async move {
        while let Some(msg) = rx_coords.recv().await {
            let parsed = parse_coordinates(&msg.data);
            coords_clone.set_coordinates(parsed);
        }
    });

    // Main control loop
    let mut last_detection_time = Instant::now();
    let timeout = Duration::from_secs(2); // Reset to center if no detection for 2 seconds
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    let mut frame_count = 0u64;
    let mut last_logged_pan = 90.0;
    let mut last_logged_tilt = 90.0;

    loop {
        interval.tick().await;
        frame_count += 1;

        // Check for ball detection coordinates
        if let Some((x1, y1, x2, y2)) = coords.get_coordinates() {
            // Check if coordinates are stale
            if !coords.is_stale(timeout) {
                let (pan_angle, tilt_angle) = controller.update(x1, y1, x2, y2);
                servo.set_pan(pan_angle)?;
                servo.set_tilt(tilt_angle)?;

                // Calculate ball center for logging
                let ball_center_x = (x1 + x2) / 2.0;
                let ball_center_y = (y1 + y2) / 2.0;
                let image_center_x = 640.0 / 2.0;
                let image_center_y = 640.0 / 2.0;
                let error_x = ball_center_x - image_center_x;
                let error_y = ball_center_y - image_center_y;
                
                // Log more frequently to debug direction issues
                let pan_diff = (pan_angle - last_logged_pan).abs();
                let tilt_diff = (tilt_angle - last_logged_tilt).abs();
                if pan_diff > 0.5 || tilt_diff > 0.5 || frame_count % 20 == 0 {
                    // Show normalized errors and direction
                    let norm_error_x = error_x / image_center_x;
                    let norm_error_y = error_y / image_center_y;
                    println!("Ball: ({:.1}, {:.1}), err: ({:.1}, {:.1}) norm: ({:.3}, {:.3}) -> Pan: {:.1}° (Δ{:.1}°), Tilt: {:.1}° (Δ{:.1}°)",
                        ball_center_x, ball_center_y, error_x, error_y, norm_error_x, norm_error_y,
                        pan_angle, pan_diff, tilt_angle, tilt_diff);
                    last_logged_pan = pan_angle;
                    last_logged_tilt = tilt_angle;
                }

                last_detection_time = Instant::now();
            } else {
                // Coordinates are stale, reset to center
                if last_detection_time.elapsed() > timeout {
                    controller.reset_to_center();
                    servo.set_pan(90.0)?;
                    servo.set_tilt(90.0)?;
                    println!("No detection for {}s, resetting to center", timeout.as_secs());
                    last_detection_time = Instant::now();
                }
            }
        } else {
            // No coordinates available, check if we should reset
            if last_detection_time.elapsed() > timeout {
                controller.reset_to_center();
                servo.set_pan(90.0)?;
                servo.set_tilt(90.0)?;
                if last_detection_time.elapsed() > timeout + Duration::from_secs(1) {
                    // Only log once per timeout period
                    println!("No detection for {}s, resetting to center", timeout.as_secs());
                    last_detection_time = Instant::now();
                }
            }
        }
    }
}
