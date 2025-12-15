use ros_wrapper::{create_topic_receiver, QosProfile, std_msgs::msg::String as StringMsg};
use software_pwm::SoftwarePwmServo;
use std::time::{Duration, Instant};
use tokio::sync::mpsc::error::TryRecvError;

// Error type for tracker operations
#[derive(Debug)]
enum TrackerError {
    Ros2(String),
    Gpio(String),
    Pwm(String),
    Io(String),
}

impl std::fmt::Display for TrackerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrackerError::Ros2(msg) => write!(f, "ROS2 error: {}", msg),
            TrackerError::Gpio(msg) => write!(f, "GPIO error: {}", msg),
            TrackerError::Pwm(msg) => write!(f, "PWM error: {}", msg),
            TrackerError::Io(msg) => write!(f, "IO error: {}", msg),
        }
    }
}

impl std::error::Error for TrackerError {}

impl From<std::io::Error> for TrackerError {
    fn from(err: std::io::Error) -> Self {
        TrackerError::Io(err.to_string())
    }
}

fn dev_node_exists(path: &str) -> bool {
    std::path::Path::new(path).exists()
}

fn any_gpiochip_exists() -> bool {
    // Check a small range; on Raspberry Pi this is typically /dev/gpiochip0.
    for i in 0..=8 {
        let path = format!("/dev/gpiochip{i}");
        if dev_node_exists(&path) {
            return true;
        }
    }
    false
}

fn gpio_preflight_diagnostics() -> String {
    // rppal uses /dev/gpiomem (Pi 0-4) or /dev/gpiomem0 (Pi 5 / RP1), and also uses /dev/gpiochipN.
    let has_gpiomem = dev_node_exists("/dev/gpiomem");
    let has_gpiomem0 = dev_node_exists("/dev/gpiomem0");
    let has_mem = dev_node_exists("/dev/mem");
    let has_gpiochip = any_gpiochip_exists();

    format!(
        "GPIO device nodes: /dev/gpiomem={}, /dev/gpiomem0={}, /dev/mem={}, /dev/gpiochipN={}",
        has_gpiomem, has_gpiomem0, has_mem, has_gpiochip
    )
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
        // Preflight: make missing /dev nodes actionable instead of a generic "No such file or directory".
        let diag = gpio_preflight_diagnostics();
        if !dev_node_exists("/dev/gpiomem")
            && !dev_node_exists("/dev/gpiomem0")
            && !dev_node_exists("/dev/mem")
        {
            return Err(TrackerError::Gpio(format!(
                "Missing GPIO memory device node(s). {}. Expected at least one of /dev/gpiomem, /dev/gpiomem0, or /dev/mem.",
                diag
            )));
        }
        if !any_gpiochip_exists() {
            return Err(TrackerError::Gpio(format!(
                "Missing GPIO character device(s). {}. Expected /dev/gpiochipN (typically /dev/gpiochip0).",
                diag
            )));
        }

        let pan_servo = SoftwarePwmServo::new(18).map_err(|e| {
            TrackerError::Pwm(format!(
                "Failed to initialize pan servo on GPIO18: {}. {}",
                e, diag
            ))
        })?;
        let tilt_servo = SoftwarePwmServo::new(19).map_err(|e| {
            TrackerError::Pwm(format!(
                "Failed to initialize tilt servo on GPIO19: {}. {}",
                e, diag
            ))
        })?;

        Ok(Self { pan_servo, tilt_servo })
    }

    fn set_pan(&self, angle: f64) -> Result<(), TrackerError> {
        let angle = angle.clamp(0.0, 180.0) as u8;
        self.pan_servo
            .set_angle(angle)
            .map_err(|e| TrackerError::Pwm(format!("Failed to set pan servo angle: {:?}", e)))?;
        Ok(())
    }

    fn set_tilt(&self, angle: f64) -> Result<(), TrackerError> {
        let angle = angle.clamp(0.0, 180.0) as u8;
        self.tilt_servo
            .set_angle(angle)
            .map_err(|e| TrackerError::Pwm(format!("Failed to set tilt servo angle: {:?}", e)))?;
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct TrackingConfig {
    image_width: u32,
    image_height: u32,
    // Camera field-of-view in degrees (used for px -> angle conversion when fx/fy not provided)
    hfov_deg: f64,
    vfov_deg: f64,
    // Optional intrinsics override (pixels). If set, these take precedence over FOV-derived values.
    fx_px: Option<f64>,
    fy_px: Option<f64>,
    cx_px: Option<f64>,
    cy_px: Option<f64>,
    // Control
    kp_pan: f64,
    kp_tilt: f64,
    kd_pan: f64,
    kd_tilt: f64,
    // Max servo speed (deg/sec)
    max_speed_deg_s: f64,
    // Deadband (degrees). If angular error is smaller than this, don't move.
    deadband_deg: f64,
    // EMA smoothing for angular error (0..1)
    ema_alpha: f64,
    // Servo limits (degrees)
    pan_min: f64,
    pan_max: f64,
    tilt_min: f64,
    tilt_max: f64,
    // Servo "center" angles (degrees). Use these to compensate for mechanical mounting offsets.
    pan_center_deg: f64,
    tilt_center_deg: f64,
    // Direction inversion flags (if servos are mounted backwards)
    invert_pan: bool,
    invert_tilt: bool,
    // Detection outlier gating
    max_jump_px_s: f64,
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
        let image_width = env_u32("TRACKER_IMAGE_WIDTH").unwrap_or(640);
        let image_height = env_u32("TRACKER_IMAGE_HEIGHT").unwrap_or(640);

        let hfov_deg = env_f64("TRACKER_HFOV_DEG").unwrap_or(70.0);
        let vfov_deg = env_f64("TRACKER_VFOV_DEG").unwrap_or(55.0);

        let fx_px = env_f64("TRACKER_FX_PX");
        let fy_px = env_f64("TRACKER_FY_PX");
        let cx_px = env_f64("TRACKER_CX_PX");
        let cy_px = env_f64("TRACKER_CY_PX");

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

        let pan_center_deg = env_f64("TRACKER_PAN_CENTER_DEG").unwrap_or(90.0);
        let tilt_center_deg = env_f64("TRACKER_TILT_CENTER_DEG").unwrap_or(90.0);

        let invert_pan = env_bool("TRACKER_INVERT_PAN").unwrap_or(false);
        let invert_tilt = env_bool("TRACKER_INVERT_TILT").unwrap_or(false);

        let max_jump_px_s = env_f64("TRACKER_MAX_JUMP_PX_S").unwrap_or(2000.0);

        Self {
            image_width,
            image_height,
            hfov_deg,
            vfov_deg,
            fx_px,
            fy_px,
            cx_px,
            cy_px,
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
            pan_center_deg,
            tilt_center_deg,
            invert_pan,
            invert_tilt,
            max_jump_px_s,
        }
    }
}

struct Ema {
    alpha: f64,
    value: Option<f64>,
}

impl Ema {
    fn new(alpha: f64) -> Self {
        Self { alpha: alpha.clamp(0.0, 1.0), value: None }
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

struct Controller {
    // Current servo positions (degrees)
    pan_angle: f64,
    tilt_angle: f64,
    last_update: Instant,

    // Camera intrinsics model (pixels)
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,

    // Config
    cfg: TrackingConfig,

    // Smoothing + derivative bookkeeping
    err_pan_ema: Ema,
    err_tilt_ema: Ema,
    last_err_pan_deg: f64,
    last_err_tilt_deg: f64,
}

impl Controller {
    fn new(cfg: TrackingConfig) -> Self {
        let w = cfg.image_width as f64;
        let h = cfg.image_height as f64;

        let cx = cfg.cx_px.unwrap_or(w / 2.0);
        let cy = cfg.cy_px.unwrap_or(h / 2.0);

        let (fx, fy) = match (cfg.fx_px, cfg.fy_px) {
            (Some(fx), Some(fy)) => (fx, fy),
            _ => {
                let hfov_rad = cfg.hfov_deg.to_radians();
                let vfov_rad = cfg.vfov_deg.to_radians();
                let fx = (w / 2.0) / (hfov_rad / 2.0).tan();
                let fy = (h / 2.0) / (vfov_rad / 2.0).tan();
                (fx, fy)
            }
        };

        Self {
            pan_angle: cfg.pan_center_deg,
            tilt_angle: cfg.tilt_center_deg,
            last_update: Instant::now(),
            fx,
            fy,
            cx,
            cy,
            err_pan_ema: Ema::new(cfg.ema_alpha),
            err_tilt_ema: Ema::new(cfg.ema_alpha),
            last_err_pan_deg: 0.0,
            last_err_tilt_deg: 0.0,
            cfg,
        }
    }

    fn reset_to_center(&mut self) {
        self.pan_angle = self.cfg.pan_center_deg;
        self.tilt_angle = self.cfg.tilt_center_deg;
        self.last_err_pan_deg = 0.0;
        self.last_err_tilt_deg = 0.0;
        self.err_pan_ema.value = None;
        self.err_tilt_ema.value = None;
    }

    fn pixel_error_to_angle_deg(&self, ball_center_x: f64, ball_center_y: f64) -> (f64, f64) {
        let dx = ball_center_x - self.cx;
        let dy = ball_center_y - self.cy;

        // Positive dx => ball to the right => positive pan error
        // Positive dy => ball below center (image y down) => positive tilt error
        let err_pan_rad = (dx / self.fx).atan();
        let err_tilt_rad = (dy / self.fy).atan();

        (err_pan_rad.to_degrees(), err_tilt_rad.to_degrees())
    }

    fn update(&mut self, ball_center_x: f64, ball_center_y: f64) -> (f64, f64) {
        let (mut err_pan_deg, mut err_tilt_deg) = self.pixel_error_to_angle_deg(ball_center_x, ball_center_y);

        if self.cfg.invert_pan {
            err_pan_deg = -err_pan_deg;
        }
        if self.cfg.invert_tilt {
            err_tilt_deg = -err_tilt_deg;
        }

        // Smooth noisy detections a bit (bbox jitter).
        let err_pan_deg = self.err_pan_ema.update(err_pan_deg);
        let err_tilt_deg = self.err_tilt_ema.update(err_tilt_deg);

        let dt = self.last_update.elapsed().as_secs_f64().max(0.01);

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

        self.pan_angle = (self.pan_angle + delta_pan).clamp(self.cfg.pan_min, self.cfg.pan_max);
        self.tilt_angle = (self.tilt_angle + delta_tilt).clamp(self.cfg.tilt_min, self.cfg.tilt_max);

        self.last_err_pan_deg = err_pan_deg;
        self.last_err_tilt_deg = err_tilt_deg;
        self.last_update = Instant::now();

        (self.pan_angle, self.tilt_angle)
    }
}

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

fn bbox_center(x1: f32, y1: f32, x2: f32, y2: f32) -> (f32, f32) {
    ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
}

fn validate_bbox(cfg: &TrackingConfig, x1: f32, y1: f32, x2: f32, y2: f32) -> Option<(f32, f32, f32, f32)> {
    // Basic shape sanity
    if !(x1.is_finite() && y1.is_finite() && x2.is_finite() && y2.is_finite()) {
        return None;
    }
    if x2 <= x1 || y2 <= y1 {
        return None;
    }

    // Image bounds sanity (allow a tiny slack due to rounding upstream)
    let w = cfg.image_width as f32;
    let h = cfg.image_height as f32;
    let slack = 2.0_f32;
    if x1 < -slack || y1 < -slack || x2 > w + slack || y2 > h + slack {
        return None;
    }

    // Avoid extremely tiny boxes (often false positives).
    let bw = x2 - x1;
    let bh = y2 - y1;
    if bw < 2.0 || bh < 2.0 {
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

async fn async_main() -> Result<(), TrackerError> {
    println!(
        "esd2025 tracker (Rust) build: {} {} (uses software_pwm via rppal::gpio, not rppal::pwm)",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    println!(
        "If you see 'test mode - ROS2 disabled' or 'Failed to create pan PWM', you are running an old binary. Rebuild with: `cargo build --release -p tracker`."
    );
    println!("Initializing tracker node...");
    println!("GPIO18: Pan servo (Left-Right)");
    println!("GPIO19: Tilt servo (Up-Down)");
    println!();

    let cfg = TrackingConfig::from_env();
    println!(
        "Tracking config: {}x{}, HFOV {:.1}°, VFOV {:.1}°, fx/fy/cx/cy overrides: {:?}/{:?}/{:?}/{:?}, KP(pan/tilt) {:.3}/{:.3}, KD(pan/tilt) {:.3}/{:.3}, max_speed {:.1}°/s, deadband {:.2}°, ema_alpha {:.2}, invert_pan {}, invert_tilt {}, max_jump {:.0}px/s",
        cfg.image_width,
        cfg.image_height,
        cfg.hfov_deg,
        cfg.vfov_deg,
        cfg.fx_px,
        cfg.fy_px,
        cfg.cx_px,
        cfg.cy_px,
        cfg.kp_pan,
        cfg.kp_tilt,
        cfg.kd_pan,
        cfg.kd_tilt,
        cfg.max_speed_deg_s,
        cfg.deadband_deg,
        cfg.ema_alpha,
        cfg.invert_pan,
        cfg.invert_tilt,
        cfg.max_jump_px_s
    );

    let mut controller = Controller::new(cfg);
    println!(
        "Camera model used: fx={:.2}px fy={:.2}px cx={:.2}px cy={:.2}px",
        controller.fx, controller.fy, controller.cx, controller.cy
    );

    let servo_disabled = std::env::var("TRACKER_DISABLE_SERVO")
        .ok()
        .map(|v| v != "0")
        .unwrap_or(false);

    let servo = if servo_disabled {
        println!("Servo output disabled (TRACKER_DISABLE_SERVO=1). Will only log computed angles.");
        None
    } else {
        let servo = ServoController::new()?;
        servo.set_pan(controller.cfg.pan_center_deg)?;
        servo.set_tilt(controller.cfg.tilt_center_deg)?;
        Some(servo)
    };

    println!("Subscribing to /ball_coords ...");
    let (mut rx_coords, _node_coords) = create_topic_receiver::<StringMsg>(
        "tracker_node",
        "/ball_coords",
        QosProfile::default(),
    )
    .map_err(|e| TrackerError::Ros2(format!("Failed to create topic receiver: {:?}", e)))?;

    println!("ROS_DOMAIN_ID: {}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!("Waiting for DDS discovery...");
    for _ in 1..=10 {
        tokio::time::sleep(Duration::from_secs(1)).await;
        print!(".");
        std::io::Write::flush(&mut std::io::stdout())?;
    }
    println!();

    // Main control loop
    let tick = Duration::from_millis(100);
    let timeout = Duration::from_secs(2);
    let mut interval = tokio::time::interval(tick);

    let mut last_detection_time = Instant::now();
    let mut last_good_center: Option<(f32, f32)> = None;
    let mut last_good_center_at = Instant::now();
    let mut rejected_count = 0u64;
    let mut frame_count = 0u64;
    let mut last_logged_pan = 90.0;
    let mut last_logged_tilt = 90.0;

    // "Latest" detection seen on the channel (even if it's "none")
    let mut latest_det: Option<(Option<(f32, f32, f32, f32)>, Instant)> = None;

    loop {
        interval.tick().await;
        frame_count += 1;

        // Drain channel so we always act on the most recent message.
        loop {
            match rx_coords.try_recv() {
                Ok(msg) => {
                    let parsed = parse_coordinates(&msg.data);
                    latest_det = Some((parsed, Instant::now()));
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    return Err(TrackerError::Ros2("ball_coords receiver disconnected".to_string()));
                }
            }
        }

        let (coords, received_at) = match latest_det {
            Some(v) => v,
            None => continue,
        };

        if received_at.elapsed() > timeout {
            if last_detection_time.elapsed() > timeout {
                controller.reset_to_center();
                if let Some(servo) = &servo {
                    servo.set_pan(controller.cfg.pan_center_deg)?;
                    servo.set_tilt(controller.cfg.tilt_center_deg)?;
                }
                println!("No detection for {}s, resetting to center", timeout.as_secs());
                last_detection_time = Instant::now();
            }
            continue;
        }

        let Some((x1, y1, x2, y2)) = coords else {
            // Explicit "none": treat as no ball.
            continue;
        };

        let Some((x1, y1, x2, y2)) = validate_bbox(&controller.cfg, x1, y1, x2, y2) else {
            rejected_count += 1;
            if rejected_count % 50 == 0 {
                eprintln!("Rejected {} detections (invalid bbox)", rejected_count);
            }
            continue;
        };

        let (cx, cy) = bbox_center(x1, y1, x2, y2);

        // Outlier gate: reject implausibly large center jumps.
        if let Some((prev_x, prev_y)) = last_good_center {
            let dt = last_good_center_at.elapsed().as_secs_f64().max(0.01);
            let max_jump = (controller.cfg.max_jump_px_s * dt) as f32;
            let dx = cx - prev_x;
            let dy = cy - prev_y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist > max_jump {
                rejected_count += 1;
                if rejected_count % 50 == 0 {
                    eprintln!(
                        "Rejected {} detections (jump {:.1}px > {:.1}px). last=({:.1},{:.1}) now=({:.1},{:.1})",
                        rejected_count,
                        dist,
                        max_jump,
                        prev_x,
                        prev_y,
                        cx,
                        cy
                    );
                }
                continue;
            }
        }

        last_good_center = Some((cx, cy));
        last_good_center_at = Instant::now();

        let (pan_angle, tilt_angle) = controller.update(cx as f64, cy as f64);
        if let Some(servo) = &servo {
            servo.set_pan(pan_angle)?;
            servo.set_tilt(tilt_angle)?;
        }
        last_detection_time = Instant::now();

        // Log periodically with pixel error, angular error, and servo angles.
        let pan_diff = (pan_angle - last_logged_pan).abs();
        let tilt_diff = (tilt_angle - last_logged_tilt).abs();
        if pan_diff > 0.5 || tilt_diff > 0.5 || frame_count % 20 == 0 {
            let image_center_x = controller.cfg.image_width as f32 / 2.0;
            let image_center_y = controller.cfg.image_height as f32 / 2.0;
            let error_x = cx - image_center_x;
            let error_y = cy - image_center_y;
            let (err_pan_deg, err_tilt_deg) = controller.pixel_error_to_angle_deg(cx as f64, cy as f64);
            println!(
                "Ball: ({:.1},{:.1}) px_err: ({:.1},{:.1}) ang_err: ({:.2}°,{:.2}°) -> Pan {:.1}° (Δ{:.1}°) Tilt {:.1}° (Δ{:.1}°)",
                cx,
                cy,
                error_x,
                error_y,
                err_pan_deg,
                err_tilt_deg,
                pan_angle,
                pan_diff,
                tilt_angle,
                tilt_diff
            );
            last_logged_pan = pan_angle;
            last_logged_tilt = tilt_angle;
        }
    }
}

