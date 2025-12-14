use ros_wrapper::{create_topic_receiver, QosProfile, std_msgs::msg::String as StringMsg};
use rppal::gpio::{Gpio, Level, OutputPin};
use rppal::pwm::{Channel, Pwm};
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};

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

// Servo control using PWM
// GPIO18: Pan (Left-Right)
// GPIO19: Tilt (Up-Down)
struct ServoController {
    // Hardware PWM for GPIO18 (PWM0)
    pan_pwm: Option<Pwm>,
    // Software PWM for GPIO19 (or hardware if available)
    tilt_pin: Option<OutputPin>,
    tilt_pwm_thread: Option<std::thread::JoinHandle<()>>,
    tilt_angle: Arc<Mutex<f64>>,
    running: Arc<Mutex<bool>>,
}

impl ServoController {
    fn new() -> Result<Self, TrackerError> {
        let gpio = Gpio::new()
            .map_err(|e| TrackerError::Gpio(format!("Failed to initialize GPIO: {:?}", e)))?;

        // GPIO18 is PWM0 (hardware PWM)
        let pan_pwm = Pwm::with_frequency(Channel::Pwm0, 50.0, 0.075, false)
            .map_err(|e| TrackerError::Pwm(format!("Failed to create pan PWM: {:?}", e)))?;
        pan_pwm.enable()
            .map_err(|e| TrackerError::Pwm(format!("Failed to enable pan PWM: {:?}", e)))?;

        // GPIO19 - use software PWM since it's not a hardware PWM pin
        let tilt_pin = gpio.get(19)
            .map_err(|e| TrackerError::Gpio(format!("Failed to get GPIO19: {:?}", e)))?
            .into_output();

        let tilt_angle = Arc::new(Mutex::new(90.0)); // Start at center
        let running = Arc::new(Mutex::new(true));
        
        // Start software PWM thread for tilt servo
        let tilt_angle_clone = Arc::clone(&tilt_angle);
        let running_clone = Arc::clone(&running);
        
        let tilt_pwm_thread = std::thread::spawn(move || {
            let period_ms = 20.0; // 50Hz = 20ms period
            let gpio = Gpio::new().ok();
            let mut tilt_pin = gpio.and_then(|g| g.get(19).ok()).map(|p| p.into_output());
            
            loop {
                let should_run = *running_clone.lock().unwrap();
                if !should_run {
                    break;
                }

                let angle = *tilt_angle_clone.lock().unwrap();
                let pulse_width_ms = Self::angle_to_pulse_width(angle);
                let pulse_width_us = (pulse_width_ms * 1000.0) as u64;
                let period_us = (period_ms * 1000.0) as u64;
                let off_time_us = period_us - pulse_width_us;

                // Generate PWM pulse
                if let Some(ref mut pin) = tilt_pin {
                    pin.set_high();
                    std::thread::sleep(Duration::from_micros(pulse_width_us));
                    pin.set_low();
                    std::thread::sleep(Duration::from_micros(off_time_us));
                } else {
                    std::thread::sleep(Duration::from_millis(20));
                }
            }
        });

        Ok(Self {
            pan_pwm: Some(pan_pwm),
            tilt_pin: None, // Pin is moved to thread
            tilt_pwm_thread: Some(tilt_pwm_thread),
            tilt_angle,
            running,
        })
    }

    // Convert angle (0-180 degrees) to pulse width in milliseconds
    // Standard servos: 1.0ms (0°) to 2.0ms (180°)
    // Some servos: 0.5ms (0°) to 2.5ms (180°)
    fn angle_to_pulse_width(angle: f64) -> f64 {
        // Clamp angle to 0-180
        let angle = angle.max(0.0).min(180.0);
        // Map 0-180 degrees to 1.0-2.0ms
        1.0 + (angle / 180.0) * 1.0
    }

    // Convert pulse width to duty cycle for hardware PWM (50Hz, 20ms period)
    fn pulse_width_to_duty_cycle(pulse_width_ms: f64) -> f64 {
        let period_ms = 20.0; // 50Hz
        (pulse_width_ms / period_ms) * 100.0
    }

    // Set pan servo position (0-180 degrees)
    fn set_pan(&self, angle: f64) -> Result<(), TrackerError> {
        let angle = angle.max(0.0).min(180.0);
        let pulse_width = Self::angle_to_pulse_width(angle);
        let duty_cycle = Self::pulse_width_to_duty_cycle(pulse_width);
        
        if let Some(ref pwm) = self.pan_pwm {
            pwm.set_duty_cycle(duty_cycle)
                .map_err(|e| TrackerError::Pwm(format!("Failed to set pan duty cycle: {:?}", e)))?;
        }
        Ok(())
    }

    // Set tilt servo position (0-180 degrees)
    fn set_tilt(&self, angle: f64) -> Result<(), TrackerError> {
        let angle = angle.max(0.0).min(180.0);
        *self.tilt_angle.lock().unwrap() = angle;
        Ok(())
    }

    fn shutdown(&mut self) {
        *self.running.lock().unwrap() = false;
        if let Some(thread) = self.tilt_pwm_thread.take() {
            thread.join().ok();
        }
        // Set servos to center position
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
}

impl Controller {
    fn new(image_width: u32, image_height: u32) -> Self {
        Self {
            kp_pan: 0.1,  // Tune these values
            kp_tilt: 0.1,
            pan_angle: 90.0,  // Start at center
            tilt_angle: 90.0,
            image_width,
            image_height,
            pan_min: 0.0,
            pan_max: 180.0,
            tilt_min: 0.0,
            tilt_max: 180.0,
            max_rate: 5.0,  // Max 5 degrees per update
            last_update: Instant::now(),
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

        // Apply P-control
        // Positive error_x means ball is to the right, need to pan right (increase angle)
        // Positive error_y means ball is below center, need to tilt down (increase angle)
        let delta_pan = self.kp_pan * normalized_error_x as f64 * 90.0; // Scale to degrees
        let delta_tilt = self.kp_tilt * normalized_error_y as f64 * 90.0;

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
    let mut servo = ServoController::new()?;
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

    // Image dimensions (should match camera resolution: 320x240)
    let image_width = 320;
    let image_height = 240;
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

    loop {
        interval.tick().await;

        // Check for ball detection coordinates
        if let Some((x1, y1, x2, y2)) = coords.get_coordinates() {
            // Check if coordinates are stale
            if !coords.is_stale(timeout) {
                let (pan_angle, tilt_angle) = controller.update(x1, y1, x2, y2);
                servo.set_pan(pan_angle)?;
                servo.set_tilt(tilt_angle)?;

                println!("Ball at ({:.1}, {:.1}) to ({:.1}, {:.1}) -> Pan: {:.1}°, Tilt: {:.1}°",
                    x1, y1, x2, y2, pan_angle, tilt_angle);

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
