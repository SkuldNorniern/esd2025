use ros_wrapper::{create_topic_receiver, QosProfile, std_msgs::msg::String as StringMsg};
use software_pwm::SoftwarePwmServo;
use std::time::{Duration, Instant};
use tokio::sync::watch;

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

#[derive(Clone, Copy, Debug)]
struct Detection {
    coords: Option<(f32, f32, f32, f32)>,
    received_at: Instant,
}

struct Ema {
    alpha: f64,
    value: Option<f64>,
}

impl Ema {
    fn new(alpha: f64) -> Self {
        let alpha = alpha.clamp(0.0, 1.0);
        Self { alpha, value: None }
    }

    fn update(&mut self, x: f64) -> f64 {
        match self.value {
            Some(prev) => {
                let v = (self.alpha * x) + ((1.0 - self.alpha) * prev);
                self.value = Some(v);
                v
            }
            None => {
                self.value = Some(x);
                x
            }
        }
    }
}

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

struct TrackingConfig {
    image_width: u32,
    image_height: u32,
    // Camera field-of-view in degrees (used for px -> angle conversion)
    hfov_deg: f64,
    vfov_deg: f64,
    // Control
    kp_pan: f64,
    kp_tilt: f64,
    kd_pan: f64,
    kd_tilt: f64,
    // Max servo speed (deg/sec)
    max_speed_deg_s: f64,
    // Deadband (degrees). If angular error is smaller than this, don't move.
    deadband_deg: f64,
    // EMA smoothing for angular error (0 = no smoothing, 1 = heavy smoothing)
    ema_alpha: f64,
    // Servo limits (degrees)
    pan_min: f64,
    pan_max: f64,
    tilt_min: f64,
    tilt_max: f64,
    // Direction inversion flags (if servos are mounted backwards)
    invert_pan: bool,
    invert_tilt: bool,
}

fn env_f64(name: &str) -> Option<f64> {
    std::env::var(name).ok()?.parse().ok()
}

fn env_u32(name: &str) -> Option<u32> {
    std::env::var(name).ok()?.parse().ok()
}

fn env_bool(name: &str) -> Option<bool> {
    match std::env::var(name).ok()?.as_str() {
        "1" | "true" | "TRUE" | "True" | "yes" | "YES" | "Yes" => Some(true),
        "0" | "false" | "FALSE" | "False" | "no" | "NO" | "No" => Some(false),
        _ => None,
    }
}

impl TrackingConfig {
    fn from_env() -> Self {
        // Defaults assume a typical webcam; override on the Pi for best accuracy.
        let image_width = env_u32("TRACKER_IMAGE_WIDTH").unwrap_or(640);
        let image_height = env_u32("TRACKER_IMAGE_HEIGHT").unwrap_or(640);

        let hfov_deg = env_f64("TRACKER_HFOV_DEG").unwrap_or(70.0);
        let vfov_deg = env_f64("TRACKER_VFOV_DEG").unwrap_or(55.0);

        // If you want "move the full error angle each tick", set KP=1.0.
        let kp_pan = env_f64("TRACKER_KP_PAN").unwrap_or(0.8);
        let kp_tilt = env_f64("TRACKER_KP_TILT").unwrap_or(0.8);

        let kd_pan = env_f64("TRACKER_KD_PAN").unwrap_or(0.0);
        let kd_tilt = env_f64("TRACKER_KD_TILT").unwrap_or(0.0);

        let max_speed_deg_s = env_f64("TRACKER_MAX_SPEED_DEG_S").unwrap_or(120.0);
        let deadband_deg = env_f64("TRACKER_DEADBAND_DEG").unwrap_or(0.4);
        let ema_alpha = env_f64("TRACKER_EMA_ALPHA").unwrap_or(0.4);

        let pan_min = env_f64("TRACKER_PAN_MIN").unwrap_or(0.0);
        let pan_max = env_f64("TRACKER_PAN_MAX").unwrap_or(180.0);
        let tilt_min = env_f64("TRACKER_TILT_MIN").unwrap_or(0.0);
        let tilt_max = env_f64("TRACKER_TILT_MAX").unwrap_or(180.0);

        let invert_pan = env_bool("TRACKER_INVERT_PAN").unwrap_or(false);
        let invert_tilt = env_bool("TRACKER_INVERT_TILT").unwrap_or(false);

        Self {
            image_width,
            image_height,
            hfov_deg,
            vfov_deg,
            kp_pan,
            kp_tilt,
            kd_pan,
            kd_tilt,
            max_speed_deg_s,
            deadband_deg,
            ema_alpha,
            pan_min,
            pan_max,
            tilt_min,
            tilt_max,
            invert_pan,
            invert_tilt,
        }
    }
}

// PID controller (starting with P-only as per README)
struct Controller {
    // Current servo positions (degrees)
    pan_angle: f64,
    tilt_angle: f64,
    last_update: Instant,

    // Camera model (focal length in pixels)
    fx: f64,
    fy: f64,

    // Config
    cfg: TrackingConfig,

    // Optional smoothing + derivative
    err_pan_ema: Ema,
    err_tilt_ema: Ema,
    last_err_pan_deg: f64,
    last_err_tilt_deg: f64,
}

impl Controller {
    fn new(cfg: TrackingConfig) -> Self {
        let w = cfg.image_width as f64;
        let h = cfg.image_height as f64;

        // Projective model:
        // fx = (w/2) / tan(hfov/2), fy = (h/2) / tan(vfov/2)
        let hfov_rad = cfg.hfov_deg.to_radians();
        let vfov_rad = cfg.vfov_deg.to_radians();
        let fx = (w / 2.0) / (hfov_rad / 2.0).tan();
        let fy = (h / 2.0) / (vfov_rad / 2.0).tan();

        Self {
            pan_angle: 90.0,  // Start at center
            tilt_angle: 90.0,
            last_update: Instant::now(),
            fx,
            fy,
            err_pan_ema: Ema::new(cfg.ema_alpha),
            err_tilt_ema: Ema::new(cfg.ema_alpha),
            last_err_pan_deg: 0.0,
            last_err_tilt_deg: 0.0,
            cfg,
        }
    }

    fn pixel_error_to_angle_deg(&self, ball_center_x: f64, ball_center_y: f64) -> (f64, f64) {
        let center_x = (self.cfg.image_width as f64) / 2.0;
        let center_y = (self.cfg.image_height as f64) / 2.0;

        let dx = ball_center_x - center_x;
        let dy = ball_center_y - center_y;

        // Positive dx => ball to the right => positive pan error
        // Positive dy => ball below center (image y down) => positive tilt error
        let err_pan_rad = (dx / self.fx).atan();
        let err_tilt_rad = (dy / self.fy).atan();

        (err_pan_rad.to_degrees(), err_tilt_rad.to_degrees())
    }

    // Update servo positions based on ball detection
    // Input: bounding box (x1, y1, x2, y2) in image coordinates
    fn update(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) -> (f64, f64) {
        // Calculate ball center
        let ball_center_x = ((x1 + x2) as f64) / 2.0;
        let ball_center_y = ((y1 + y2) as f64) / 2.0;

        // Convert pixel offset to angular error using camera FOV.
        // This is the key to "accurately point at the ball".
        let (mut err_pan_deg, mut err_tilt_deg) = self.pixel_error_to_angle_deg(ball_center_x, ball_center_y);

        if self.cfg.invert_pan {
            err_pan_deg = -err_pan_deg;
        }
        if self.cfg.invert_tilt {
            err_tilt_deg = -err_tilt_deg;
        }

        // Smooth noisy detections a bit (bounding box jitter).
        let err_pan_deg = self.err_pan_ema.update(err_pan_deg);
        let err_tilt_deg = self.err_tilt_ema.update(err_tilt_deg);

        let elapsed = self.last_update.elapsed().as_secs_f64();
        let dt = elapsed.max(0.01);

        // Deadband to prevent servo buzz when nearly centered.
        let mut delta_pan = 0.0;
        let mut delta_tilt = 0.0;

        if err_pan_deg.abs() >= self.cfg.deadband_deg {
            let derr = (err_pan_deg - self.last_err_pan_deg) / dt;
            delta_pan = (self.cfg.kp_pan * err_pan_deg) + (self.cfg.kd_pan * derr);
        }

        if err_tilt_deg.abs() >= self.cfg.deadband_deg {
            let derr = (err_tilt_deg - self.last_err_tilt_deg) / dt;
            delta_tilt = (self.cfg.kp_tilt * err_tilt_deg) + (self.cfg.kd_tilt * derr);
        }

        // Rate limiting (deg/sec)
        let max_delta = self.cfg.max_speed_deg_s * dt;
        delta_pan = delta_pan.clamp(-max_delta, max_delta);
        delta_tilt = delta_tilt.clamp(-max_delta, max_delta);

        // Update angles
        self.pan_angle += delta_pan;
        self.tilt_angle += delta_tilt;

        // Clamp to limits
        self.pan_angle = self.pan_angle.max(self.cfg.pan_min).min(self.cfg.pan_max);
        self.tilt_angle = self.tilt_angle.max(self.cfg.tilt_min).min(self.cfg.tilt_max);

        self.last_err_pan_deg = err_pan_deg;
        self.last_err_tilt_deg = err_tilt_deg;
        self.last_update = Instant::now();

        (self.pan_angle, self.tilt_angle)
    }

    // Reset to center position
    fn reset_to_center(&mut self) {
        self.pan_angle = 90.0;
        self.tilt_angle = 90.0;
    }
}

// Parse coordinates string: "x1,y1,x2,y2" or "none"
fn parse_coordinates(s: &str) -> Option<(f32, f32, f32, f32)> {
    if s == "none" {
        return None;
    }

    let mut it = s.split(',');
    let x1 = it.next()?.parse().ok()?;
    let y1 = it.next()?.parse().ok()?;
    let x2 = it.next()?.parse().ok()?;
    let y2 = it.next()?.parse().ok()?;
    if it.next().is_some() {
        return None;
    }
    Some((x1, y1, x2, y2))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Avoid `#[tokio::main]` to keep tokio's proc-macro feature out of this crate.
    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(async_main())?;
    Ok(())
}

async fn async_main() -> Result<(), Box<dyn std::error::Error>> {
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

    let cfg = TrackingConfig::from_env();
    println!(
        "Tracking config: {}x{}, HFOV {:.1}°, VFOV {:.1}°, KP(pan/tilt) {:.3}/{:.3}, KD(pan/tilt) {:.3}/{:.3}, max_speed {:.1}°/s, deadband {:.2}°, ema_alpha {:.2}, invert_pan {}, invert_tilt {}",
        cfg.image_width,
        cfg.image_height,
        cfg.hfov_deg,
        cfg.vfov_deg,
        cfg.kp_pan,
        cfg.kp_tilt,
        cfg.kd_pan,
        cfg.kd_tilt,
        cfg.max_speed_deg_s,
        cfg.deadband_deg,
        cfg.ema_alpha,
        cfg.invert_pan,
        cfg.invert_tilt
    );

    let mut controller = Controller::new(cfg);

    let (det_tx, det_rx) = watch::channel(Detection {
        coords: None,
        received_at: Instant::now(),
    });

    tokio::spawn(async move {
        while let Some(msg) = rx_coords.recv().await {
            let parsed = parse_coordinates(&msg.data);
            let _ = det_tx.send(Detection {
                coords: parsed,
                received_at: Instant::now(),
            });
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
        let det = *det_rx.borrow();
        if det.received_at.elapsed() <= timeout {
            if let Some((x1, y1, x2, y2)) = det.coords {
                let (pan_angle, tilt_angle) = controller.update(x1, y1, x2, y2);
                servo.set_pan(pan_angle)?;
                servo.set_tilt(tilt_angle)?;

                let ball_center_x = (x1 + x2) / 2.0;
                let ball_center_y = (y1 + y2) / 2.0;

                // Log periodically with pixel error and current servo angles.
                let pan_diff = (pan_angle - last_logged_pan).abs();
                let tilt_diff = (tilt_angle - last_logged_tilt).abs();
                if pan_diff > 0.5 || tilt_diff > 0.5 || frame_count % 20 == 0 {
                    let image_center_x = controller.cfg.image_width as f32 / 2.0;
                    let image_center_y = controller.cfg.image_height as f32 / 2.0;
                    let error_x = ball_center_x - image_center_x;
                    let error_y = ball_center_y - image_center_y;
                    println!(
                        "Ball: ({:.1},{:.1}) px_err: ({:.1},{:.1}) -> Pan {:.1}° (Δ{:.1}°) Tilt {:.1}° (Δ{:.1}°)",
                        ball_center_x,
                        ball_center_y,
                        error_x,
                        error_y,
                        pan_angle,
                        pan_diff,
                        tilt_angle,
                        tilt_diff
                    );
                    last_logged_pan = pan_angle;
                    last_logged_tilt = tilt_angle;
                }

                last_detection_time = Instant::now();
            }
        } else if last_detection_time.elapsed() > timeout {
            controller.reset_to_center();
            servo.set_pan(90.0)?;
            servo.set_tilt(90.0)?;
            println!("No detection for {}s, resetting to center", timeout.as_secs());
            last_detection_time = Instant::now();
        }
    }
}

