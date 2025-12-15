#!/usr/bin/env python3
"""
Ball Tracker Node - Python implementation
Subscribes to /ball_coords topic and controls pan/tilt servos to track the ball
GPIO18: Pan servo (Left-Right)
GPIO19: Tilt servo (Up-Down)

Requirements:
- ROS2 installed and sourced (source /opt/ros/<distro>/setup.bash)
- rclpy installed: sudo apt install ros-<distro>-rclpy
- gpiozero: pip install gpiozero pigpio
- pigpio daemon running: sudo pigpiod
"""

import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    RCLPY_AVAILABLE = True
except ImportError as e:
    RCLPY_AVAILABLE = False
    print(f"Error: rclpy not available: {e}")
    print("To install ROS2 Python packages:")
    print("  1. Source ROS2: source /opt/ros/<distro>/setup.bash")
    print("  2. Install rclpy: sudo apt install ros-<distro>-rclpy")
    print("  3. Common distros: humble, foxy, galactic, iron")
    print()
    print("Alternatively, use the Rust version: cargo run --release -p tracker")
    sys.exit(1)

import math
import time
import os
from typing import Optional, Tuple
import signal

# Try to import GPIO libraries (Raspberry Pi only)
try:
    from gpiozero import Servo
    from gpiozero.pins.pigpio import PiGPIOFactory
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: gpiozero not available. Servo control will be disabled.")
    print("  Install with: pip install gpiozero pigpio")


class ServoController:
    """Control pan and tilt servos using GPIO"""
    
    def __init__(self):
        if not GPIO_AVAILABLE:
            raise RuntimeError("GPIO libraries not available")
        
        # Initialize pigpio factory for better PWM control
        factory = PiGPIOFactory()
        
        # GPIO18: Pan servo (Left-Right)
        # GPIO19: Tilt servo (Up-Down)
        # Servo range: -1.0 (0°) to 1.0 (180°)
        # We'll map 0-180 degrees to -1.0 to 1.0
        self.pan_servo = Servo(18, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.tilt_servo = Servo(19, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        
        # Set initial position to configured mechanical center (degrees).
        # These env vars match the Rust tracker config.
        pan_center = float(os.environ.get("TRACKER_PAN_CENTER_DEG", "90.0"))
        tilt_center = float(os.environ.get("TRACKER_TILT_CENTER_DEG", "90.0"))
        self.set_pan(pan_center)
        self.set_tilt(tilt_center)
    
    def set_pan(self, angle: float) -> None:
        """Set pan servo position (0-180 degrees)"""
        angle = max(0.0, min(180.0, angle))
        # Map 0-180 degrees to -1.0 to 1.0 for gpiozero Servo
        # 0° -> -1.0, 90° -> 0.0, 180° -> 1.0
        servo_value = (angle - 90.0) / 90.0
        self.pan_servo.value = servo_value
    
    def set_tilt(self, angle: float) -> None:
        """Set tilt servo position (0-180 degrees)"""
        angle = max(0.0, min(180.0, angle))
        # Map 0-180 degrees to -1.0 to 1.0 for gpiozero Servo
        servo_value = (angle - 90.0) / 90.0
        self.tilt_servo.value = servo_value
    
    def shutdown(self) -> None:
        """Set servos to center position before shutdown"""
        pan_center = float(os.environ.get("TRACKER_PAN_CENTER_DEG", "90.0"))
        tilt_center = float(os.environ.get("TRACKER_TILT_CENTER_DEG", "90.0"))
        self.set_pan(pan_center)
        self.set_tilt(tilt_center)
        time.sleep(0.1)  # Give servos time to move


class Controller:
    """PD controller for ball tracking (angle-domain control)."""

    def __init__(self, image_width: int, image_height: int):
        self.image_width = float(image_width)
        self.image_height = float(image_height)

        # Camera model / mapping: prefer fx/fy if provided (better than HFOV/VFOV).
        self.hfov_deg = float(os.environ.get("TRACKER_HFOV_DEG", "70.0"))
        self.vfov_deg = float(os.environ.get("TRACKER_VFOV_DEG", "55.0"))
        self.fx_px = self._env_float("TRACKER_FX_PX", 0.0)
        self.fy_px = self._env_float("TRACKER_FY_PX", 0.0)

        # Image center (if intrinsics known, use them).
        self.cx_px = self._env_float("TRACKER_CX_PX", self.image_width / 2.0)
        self.cy_px = self._env_float("TRACKER_CY_PX", self.image_height / 2.0)

        # Control gains (units: degrees of servo change per degree of angular error).
        # REDUCED for slow, smooth tracking that camera can follow
        # Lower Kp = gentler response, less overshoot, easier for camera to track
        self.kp_pan = self._env_float("TRACKER_KP_PAN", 0.5)
        self.kp_tilt = self._env_float("TRACKER_KP_TILT", 0.5)
        self.kd_pan = self._env_float("TRACKER_KD_PAN", 0.05)
        self.kd_tilt = self._env_float("TRACKER_KD_TILT", 0.05)

        # Physical servo-to-laser mapping (verified from measurements):
        #   Pan INCREASE = Laser moves RIGHT (X increases)
        #   Pan DECREASE = Laser moves LEFT (X decreases)
        #   Tilt INCREASE = Laser moves DOWN (Y increases)
        #   Tilt DECREASE = Laser moves UP (Y decreases)
        #
        # With this mapping:
        #   - Positive X error (ball right of laser) needs positive pan delta
        #   - Positive Y error (ball below laser) needs positive tilt delta
        # So both inversions should be False by default.
        # Override with TRACKER_INVERT_PAN=1 or TRACKER_INVERT_TILT=1 if your
        # physical setup has the opposite relationship.
        self.invert_pan = self._env_bool("TRACKER_INVERT_PAN", False)
        self.invert_tilt = self._env_bool("TRACKER_INVERT_TILT", False)

        self.pan_center = self._env_float("TRACKER_PAN_CENTER_DEG", 90.0)
        self.tilt_center = self._env_float("TRACKER_TILT_CENTER_DEG", 90.0)

        # Current servo positions (degrees)
        self.pan_angle = float(self.pan_center)
        self.tilt_angle = float(self.tilt_center)

        # Servo limits (degrees)
        self.pan_min = 0.0
        self.pan_max = 180.0
        self.tilt_min = 0.0
        self.tilt_max = 180.0

        # Servo speed limit (deg/sec) - SLOW for camera to track laser
        # At 30Hz control rate, 30°/s = 1° per frame, 60°/s = 2° per frame
        self.max_speed_deg_s = self._env_float("TRACKER_MAX_SPEED_DEG_S", 30.0)

        # Safety clamps:
        # - If the control loop is delayed (ROS callback scheduling, pigpio latency, CPU load),
        #   `dt` can jump from ~0.05s (20Hz) to 0.15s+ and the rate limiter would allow a
        #   proportionally larger per-tick move (e.g. 180 deg/s * 0.15s = 27 deg), which feels
        #   "aggressive". Clamp the effective dt used for rate limiting and the D-term.
        self.max_dt_s = self._env_float("TRACKER_MAX_DT_S", 0.08)
        # - Additionally clamp the *per tick* movement (degrees), independent of dt.
        # CRITICAL: Must be small enough for camera to track laser between frames
        # At 30Hz, max_step=2.0° means laser moves ~20-40px per frame (trackable)
        self.max_step_deg = self._env_float("TRACKER_MAX_STEP_DEG", 2.0)

        # Small deadband in angular domain to prevent buzzing
        self.deadband_deg = self._env_float("TRACKER_DEADBAND_DEG", 0.3)

        # Optional EMA smoothing of input points (0..1)
        # Lower alpha = heavier smoothing, reduces jitter, helps camera track laser
        # Higher alpha = more responsive but jittery
        self.ema_alpha = self._env_float("TRACKER_EMA_ALPHA", 0.3)

        self.last_update = time.time()
        self.last_err_pan_deg = 0.0
        self.last_err_tilt_deg = 0.0
        self._ball_ema: Optional[Tuple[float, float]] = None
        self._laser_ema: Optional[Tuple[float, float]] = None
        # Internal estimate of laser dot position in pixels.
        # If `/laser_pos` is not available, we keep the last estimate and update it
        # based on the commanded servo angles (open-loop).
        self._laser_est_px: Optional[Tuple[float, float]] = None
        
        # Track error magnitude for adaptive control
        self._error_history: list = []
        self._stuck_counter = 0
        
        # Adaptive tuning state - DISABLED by default for slow tracking mode
        # Enable with: export TRACKER_ADAPTIVE=1
        self._adaptive_enabled = self._env_bool("TRACKER_ADAPTIVE", False)
        self._kp_pan_adaptive = self.kp_pan
        self._kp_tilt_adaptive = self.kp_tilt
        self._kd_pan_adaptive = self.kd_pan
        self._kd_tilt_adaptive = self.kd_tilt
        
        # Performance tracking for auto-tuning
        self._convergence_history: list = []  # Track if error is decreasing
        self._oscillation_detector_pan: list = []  # Detect sign flips (oscillation)
        self._oscillation_detector_tilt: list = []
        self._last_adjustment_time = time.time()
        self._adjustment_interval = 2.0  # Adjust gains every 2 seconds
        
        # Integral terms for steady-state error correction (I in PID)
        self._integral_pan_deg = 0.0
        self._integral_tilt_deg = 0.0
        self._ki_pan = self._env_float("TRACKER_KI_PAN", 0.0)
        self._ki_tilt = self._env_float("TRACKER_KI_TILT", 0.0)
        self._integral_max_deg = 10.0  # Anti-windup limit
        
        # Tuning info for logging
        self._last_tune_info = {
            'avg_error': 0.0,
            'pan_osc': False,
            'tilt_osc': False,
            'kp_changed': False
        }

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        try:
            return float(os.environ.get(name, str(default)))
        except ValueError:
            return default

    @staticmethod
    def _env_bool(name: str, default: bool) -> bool:
        v = os.environ.get(name)
        if v is None:
            return default
        return v.strip() not in ("0", "false", "False", "no", "NO")

    def _ema(self, prev: Optional[Tuple[float, float]], x: float, y: float) -> Tuple[float, float]:
        if prev is None:
            return (x, y)
        a = max(0.0, min(1.0, self.ema_alpha))
        return (prev[0] * (1.0 - a) + x * a, prev[1] * (1.0 - a) + y * a)

    def _px_to_deg_x(self, err_px: float) -> float:
        if self.fx_px > 0.0:
            return math.degrees(math.atan(err_px / self.fx_px))
        # Small-angle approximation from HFOV: degrees per pixel.
        return err_px * (self.hfov_deg / self.image_width)

    def _px_to_deg_y(self, err_px: float) -> float:
        if self.fy_px > 0.0:
            return math.degrees(math.atan(err_px / self.fy_px))
        return err_px * (self.vfov_deg / self.image_height)

    def _deg_to_px_x(self, deg: float) -> float:
        if self.fx_px > 0.0:
            return math.tan(math.radians(deg)) * self.fx_px
        return deg * (self.image_width / self.hfov_deg)

    def _deg_to_px_y(self, deg: float) -> float:
        if self.fy_px > 0.0:
            return math.tan(math.radians(deg)) * self.fy_px
        return deg * (self.image_height / self.vfov_deg)

    def _clamp_px(self, x: float, y: float) -> Tuple[float, float]:
        x = max(0.0, min(self.image_width - 1.0, x))
        y = max(0.0, min(self.image_height - 1.0, y))
        return (x, y)
    
    def _auto_tune_gains(self) -> None:
        """Automatically adjust control gains based on tracking performance."""
        if len(self._error_history) < 8:
            return
        
        # Analyze recent error trend
        recent_errors = self._error_history[-8:]
        avg_error = sum(recent_errors) / len(recent_errors)
        error_trend = recent_errors[-1] - recent_errors[0]  # Positive = getting worse
        error_variance = sum((e - avg_error)**2 for e in recent_errors) / len(recent_errors)
        
        # Detect oscillations (sign flips in error signal)
        pan_oscillating = self._detect_oscillation(self._oscillation_detector_pan)
        tilt_oscillating = self._detect_oscillation(self._oscillation_detector_tilt)
        
        # Tune Pan gains - adjusted thresholds for real-world tracking
        if pan_oscillating:
            # Reduce gains to dampen oscillation
            self._kp_pan_adaptive *= 0.90
            self._kd_pan_adaptive *= 1.05  # Gentle damping increase
        elif avg_error > 4.0 and error_trend >= 0:
            # Error is large and not improving - increase Kp gradually
            self._kp_pan_adaptive = min(2.0, self._kp_pan_adaptive * 1.10)
            # Enable integral term if persistently stuck
            if self._ki_pan == 0.0 and len(self._error_history) >= 10:
                self._ki_pan = 0.03
        elif avg_error < 1.0:
            # Tracking well - slightly reduce gains for smoothness
            self._kp_pan_adaptive = max(self.kp_pan * 0.7, self._kp_pan_adaptive * 0.99)
        
        # Tune Tilt gains (same logic)
        if tilt_oscillating:
            self._kp_tilt_adaptive *= 0.90
            self._kd_tilt_adaptive *= 1.05
        elif avg_error > 4.0 and error_trend >= 0:
            self._kp_tilt_adaptive = min(2.0, self._kp_tilt_adaptive * 1.10)
            if self._ki_tilt == 0.0 and len(self._error_history) >= 10:
                self._ki_tilt = 0.03
        elif avg_error < 1.0:
            self._kp_tilt_adaptive = max(self.kp_tilt * 0.7, self._kp_tilt_adaptive * 0.99)
        
        # Bounds checking
        prev_kp_pan = self._kp_pan_adaptive
        prev_kp_tilt = self._kp_tilt_adaptive
        
        self._kp_pan_adaptive = max(0.3, min(3.0, self._kp_pan_adaptive))
        self._kp_tilt_adaptive = max(0.3, min(3.0, self._kp_tilt_adaptive))
        self._kd_pan_adaptive = max(0.0, min(0.3, self._kd_pan_adaptive))
        self._kd_tilt_adaptive = max(0.0, min(0.3, self._kd_tilt_adaptive))
        self._ki_pan = max(0.0, min(0.2, self._ki_pan))
        self._ki_tilt = max(0.0, min(0.2, self._ki_tilt))
        
        # Store tuning info for logging
        self._last_tune_info = {
            'avg_error': avg_error,
            'pan_osc': pan_oscillating,
            'tilt_osc': tilt_oscillating,
            'kp_changed': abs(self._kp_pan_adaptive - prev_kp_pan) > 0.01 or 
                         abs(self._kp_tilt_adaptive - prev_kp_tilt) > 0.01
        }
    
    def _detect_oscillation(self, error_history: list) -> bool:
        """Detect if the error signal is oscillating (repeated sign changes)."""
        if len(error_history) < 6:
            return False
        
        # Count sign changes in recent history
        sign_changes = 0
        for i in range(1, len(error_history)):
            if error_history[i] * error_history[i-1] < 0:
                sign_changes += 1
        
        # If more than half the samples are sign changes, we're oscillating
        return sign_changes >= (len(error_history) - 1) // 2

    def update(
        self,
        ball_center_x: float,
        ball_center_y: float,
        ball_width: float,
        ball_height: float,
        laser_x: Optional[float],
        laser_y: Optional[float],
        now_s: float,
    ) -> Tuple[float, float]:
        """Update servo positions based on ball and laser positions in pixels."""

        # Smooth inputs (helps noisy detections / laser dot jitter).
        self._ball_ema = self._ema(self._ball_ema, ball_center_x, ball_center_y)
        ball_center_x, ball_center_y = self._ball_ema

        laser_measured = (laser_x is not None) and (laser_y is not None)

        # Laser position:
        # - If a measurement is provided, use it (smoothed) and update our estimate.
        # - If no measurement is provided, keep the last estimate and update it based on the
        #   *commanded servo movement* (dead-reckoning). This matches the desired behavior:
        #   "keep the latest position and add based on the angle movement", and avoids large
        #   jumps if the absolute angle->pixel model is imperfect.
        if laser_measured:
            self._laser_ema = self._ema(self._laser_ema, laser_x, laser_y)
            laser_x, laser_y = self._laser_ema
            self._laser_est_px = (laser_x, laser_y)
        else:
            if self._laser_est_px is None:
                # Initialize estimate to center on first use.
                self._laser_est_px = (self.cx_px, self.cy_px)
            laser_x, laser_y = self._laser_est_px

        dt_raw = max(0.001, now_s - self.last_update)
        dt = min(dt_raw, max(0.001, self.max_dt_s))

        err_px_x = ball_center_x - laser_x
        err_px_y = ball_center_y - laser_y

        # If the *measured* laser dot overlaps the detected ball region, stop moving.
        #
        # Important: do NOT do this when the laser is estimated (open-loop), because the estimate
        # can be wrong; freezing based on a wrong estimate looks like "tracker does nothing".
        #
        # We use a tighter "center hit" region than the full bbox so the tracker still recenters
        # the dot on the ball instead of stopping anywhere inside a large detector box.
        # Reduced tolerance to require more precise alignment before stopping
        # 
        # Disable overlap check if we've been stuck (allows controller to keep trying)
        if laser_measured and not hasattr(self, '_stuck_counter'):
            self._stuck_counter = 0
        
        if laser_measured and self._stuck_counter < 10:
            tol = min(5.0, max(1.5, 0.15 * min(float(ball_width), float(ball_height))))
            if abs(err_px_x) <= tol and abs(err_px_y) <= tol:
                # Prevent derivative kick when we lock.
                self.last_update = now_s
                self.last_err_pan_deg = 0.0
                self.last_err_tilt_deg = 0.0
                self._error_history = []
                self._stuck_counter = 0
                return (self.pan_angle, self.tilt_angle)

        # Convert pixel error to angular error (degrees).
        err_pan_deg = self._px_to_deg_x(err_px_x)
        err_tilt_deg = self._px_to_deg_y(err_px_y)

        # Deadband in angular domain
        if abs(err_pan_deg) < self.deadband_deg:
            err_pan_deg = 0.0
        if abs(err_tilt_deg) < self.deadband_deg:
            err_tilt_deg = 0.0

        derr_pan = (err_pan_deg - self.last_err_pan_deg) / dt
        derr_tilt = (err_tilt_deg - self.last_err_tilt_deg) / dt

        # Adaptive gain tuning
        error_magnitude = math.sqrt(err_pan_deg**2 + err_tilt_deg**2)
        self._error_history.append(error_magnitude)
        if len(self._error_history) > 10:
            self._error_history.pop(0)
        
        # Track oscillations (sign changes in error)
        self._oscillation_detector_pan.append(err_pan_deg)
        self._oscillation_detector_tilt.append(err_tilt_deg)
        if len(self._oscillation_detector_pan) > 8:
            self._oscillation_detector_pan.pop(0)
        if len(self._oscillation_detector_tilt) > 8:
            self._oscillation_detector_tilt.pop(0)
        
        # Integral term accumulation (with anti-windup)
        if abs(err_pan_deg) > self.deadband_deg:
            self._integral_pan_deg += err_pan_deg * dt
            self._integral_pan_deg = max(-self._integral_max_deg, 
                                        min(self._integral_max_deg, self._integral_pan_deg))
        else:
            self._integral_pan_deg *= 0.95  # Decay when in deadband
            
        if abs(err_tilt_deg) > self.deadband_deg:
            self._integral_tilt_deg += err_tilt_deg * dt
            self._integral_tilt_deg = max(-self._integral_max_deg, 
                                         min(self._integral_max_deg, self._integral_tilt_deg))
        else:
            self._integral_tilt_deg *= 0.95
        
        # Auto-tune gains periodically
        if self._adaptive_enabled and (now_s - self._last_adjustment_time) >= self._adjustment_interval:
            self._auto_tune_gains()
            self._last_adjustment_time = now_s
        
        # Detect if we're stuck (large error not improving)
        if len(self._error_history) >= 10:
            avg_error = sum(self._error_history) / len(self._error_history)
            if avg_error > 4.0:
                self._stuck_counter += 1
            elif avg_error < 1.5:
                self._stuck_counter = max(0, self._stuck_counter - 1)
        
        # Use adaptive gains
        kp_pan_effective = self._kp_pan_adaptive
        kp_tilt_effective = self._kp_tilt_adaptive
        kd_pan_effective = self._kd_pan_adaptive
        kd_tilt_effective = self._kd_tilt_adaptive
        
        # PID control law
        delta_pan = (kp_pan_effective * err_pan_deg + 
                     kd_pan_effective * derr_pan + 
                     self._ki_pan * self._integral_pan_deg)
        delta_tilt = (kp_tilt_effective * err_tilt_deg + 
                      kd_tilt_effective * derr_tilt + 
                      self._ki_tilt * self._integral_tilt_deg)

        if self.invert_pan:
            delta_pan = -delta_pan
        if self.invert_tilt:
            delta_tilt = -delta_tilt

        # Rate limit (deg/sec)
        max_delta = self.max_speed_deg_s * dt
        delta_pan = max(-max_delta, min(max_delta, delta_pan))
        delta_tilt = max(-max_delta, min(max_delta, delta_tilt))

        # Per-tick clamp (deg)
        delta_pan = max(-self.max_step_deg, min(self.max_step_deg, delta_pan))
        delta_tilt = max(-self.max_step_deg, min(self.max_step_deg, delta_tilt))

        # IMPORTANT:
        # Use the *actual applied* servo movement after limit clamping.
        # If we dead-reckon the laser estimate using the commanded delta while the servo is
        # saturated (e.g. pan already at 0°), the estimate can "move" even though the hardware
        # cannot. That leads to fake small pixel error in logs while the real laser dot is
        # off-screen.
        prev_pan = self.pan_angle
        prev_tilt = self.tilt_angle
        new_pan = max(self.pan_min, min(self.pan_max, self.pan_angle + delta_pan))
        new_tilt = max(self.tilt_min, min(self.tilt_max, self.tilt_angle + delta_tilt))
        applied_delta_pan = new_pan - prev_pan
        applied_delta_tilt = new_tilt - prev_tilt
        self.pan_angle = new_pan
        self.tilt_angle = new_tilt

        # Open-loop laser estimate update (dead-reckoning) when no measurement is present.
        # Sign convention (with default invert_pan=False, invert_tilt=False):
        # - Increasing pan moves laser RIGHT => x increases
        # - Increasing tilt moves laser DOWN => y increases
        # If invert flags are True, the relationship is flipped.
        if not laser_measured:
            if self._laser_est_px is None:
                self._laser_est_px = (self.cx_px, self.cy_px)
            est_x, est_y = self._laser_est_px
            sign_pan = -1.0 if self.invert_pan else 1.0
            sign_tilt = -1.0 if self.invert_tilt else 1.0
            est_x += sign_pan * self._deg_to_px_x(applied_delta_pan)
            est_y += sign_tilt * self._deg_to_px_y(applied_delta_tilt)
            self._laser_est_px = self._clamp_px(est_x, est_y)

        self.last_update = now_s
        self.last_err_pan_deg = err_pan_deg
        self.last_err_tilt_deg = err_tilt_deg

        return (self.pan_angle, self.tilt_angle)
    
    def reset_to_center(self) -> None:
        """Reset to center position"""
        self.pan_angle = float(self.pan_center)
        self.tilt_angle = float(self.tilt_center)
        
        # Reset adaptive state
        self._error_history = []
        self._oscillation_detector_pan = []
        self._oscillation_detector_tilt = []
        self._integral_pan_deg = 0.0
        self._integral_tilt_deg = 0.0
        self._stuck_counter = 0

    def reset_laser_estimate_to_center(self) -> None:
        """
        Reset the internal laser position estimate to the image center.

        Why this exists:
        - When `/laser_pos` is unavailable (e.g. the laser dot is out of frame),
          the controller uses an open-loop estimate that can drift and/or end up
          pinned at the image boundary due to clamping.
        - In those cases, it is safer to explicitly re-seed the estimate so that
          reacquisition logic (in the node) can bring the system back to a known state.
        """
        self._laser_ema = None
        self._laser_est_px = (self.cx_px, self.cy_px)


def parse_coordinates(s: str) -> Optional[Tuple[float, float, float, float]]:
    """Parse coordinates string: 'x1,y1,x2,y2' or 'none'"""
    if s == "none":
        return None
    
    parts = s.split(',')
    if len(parts) != 4:
        return None
    
    try:
        x1 = float(parts[0])
        y1 = float(parts[1])
        x2 = float(parts[2])
        y2 = float(parts[3])
        return (x1, y1, x2, y2)
    except ValueError:
        return None


def parse_laser_position(s: str) -> Optional[Tuple[float, float]]:
    """Parse laser position string: 'x,y' or 'none'"""
    if s == "none":
        return None
    
    parts = s.split(',')
    if len(parts) != 2:
        return None
    
    try:
        x = float(parts[0])
        y = float(parts[1])
        return (x, y)
    except ValueError:
        return None


class BallTrackerNode(Node):
    """ROS2 node for ball tracking"""
    
    def __init__(self):
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy not available")
        super().__init__('tracker_node')
        
        # Print ROS environment that commonly breaks discovery (especially cross-device).
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        ros_localhost_only = os.environ.get("ROS_LOCALHOST_ONLY", "0")
        ros_auto_discovery_range = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
        rmw_impl = os.environ.get("RMW_IMPLEMENTATION", "(default)")
        print("ROS environment:")
        print(f"  ROS_DOMAIN_ID: {ros_domain_id}")
        print(f"  ROS_LOCALHOST_ONLY: {ros_localhost_only}")
        print(f"  ROS_AUTOMATIC_DISCOVERY_RANGE: {ros_auto_discovery_range}")
        print(f"  RMW_IMPLEMENTATION: {rmw_impl}")
        if ros_localhost_only not in ("0", "", "false", "False", "no", "NO"):
            print("WARNING: ROS_LOCALHOST_ONLY is set. This blocks cross-device discovery.")
            print("  Fix: `unset ROS_LOCALHOST_ONLY` or `export ROS_LOCALHOST_ONLY=0`")
        if ros_auto_discovery_range in ("LOCALHOST", "OFF"):
            print(f"WARNING: ROS_AUTOMATIC_DISCOVERY_RANGE={ros_auto_discovery_range} may block discovery.")
            print("  Fix: `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`")
        print()

        print("Initializing tracker node...")
        print("GPIO18: Pan servo (Left-Right)")
        print("GPIO19: Tilt servo (Up-Down)")
        print()
        
        # Initialize servo controller
        if GPIO_AVAILABLE:
            try:
                self.servo = ServoController()
                print("Servo controller initialized")
            except Exception as e:
                print(f"Error initializing servos: {e}")
                print("Continuing without servo control...")
                self.servo = None
        else:
            print("GPIO not available, running in simulation mode")
            self.servo = None
        
        # Image dimensions (should match camera resolution: 640x480)
        # Get from environment or use detected size from first message
        self.image_width = int(os.environ.get("TRACKER_IMAGE_WIDTH", "640"))
        self.image_height = int(os.environ.get("TRACKER_IMAGE_HEIGHT", "480"))
        self.controller = Controller(self.image_width, self.image_height)
        print(f"  Using image dimensions: {self.image_width}x{self.image_height}")
        print(f"  (Set TRACKER_IMAGE_WIDTH/HEIGHT env vars to override)")

        # Reject one-frame detector glitches that jump too far (px/sec).
        # This is extremely common with bounding-box detectors and causes servo snaps.
        # Increased to allow faster ball movements without rejecting valid detections
        try:
            self.max_jump_px_s = float(os.environ.get("TRACKER_MAX_JUMP_PX_S", "3500.0"))
        except ValueError:
            self.max_jump_px_s = 3500.0
        self._last_ball_center: Optional[Tuple[float, float]] = None
        self._last_ball_time_s: Optional[float] = None
        
        # Ball coordinates state
        self.coords: Optional[Tuple[float, float, float, float]] = None
        self.last_coords_update = time.time()
        
        # Laser position state (from ROS topic)
        self.laser_pos: Optional[Tuple[float, float]] = None
        self.last_laser_update = time.time()
        # If no `/laser_pos` update for this long, fall back to open-loop estimation.
        self.laser_timeout = 1.0

        # Estimation (no valid laser measurement) should only last a few frames. If it goes
        # longer, assume the laser is in a weird place and reset. Fixed: 4 frames max.
        self.laser_estimated_max_steps = 4
        self._laser_estimated_steps: int = 0
        self._laser_recovery_active = False
        self._last_laser_recovery_log_s = 0.0
        
        # Timeout for reset (2 seconds)
        self.timeout = 2.0
        self.last_detection_time = time.time()
        
        # Use explicit QoS to avoid silent QoS incompatibilities.
        # ball_detect publishes with r2r defaults (typically RELIABLE/VOLATILE).
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create subscriber for ball coordinates
        print("Subscribing to ball detection topic...")
        self.subscription = self.create_subscription(
            String,
            '/ball_coords',
            self.coords_callback,
            qos
        )
        
        print("Subscriber created successfully")
        print("  Topic: /ball_coords")
        print("  Message type: std_msgs/String")
        print("  Format: 'x1,y1,x2,y2' or 'none'")
        print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
        print()
        
        # Create subscriber for laser position (for real-time calibration)
        print("Subscribing to laser position topic...")
        self.laser_subscription = self.create_subscription(
            String,
            '/laser_pos',
            self.laser_callback,
            qos
        )
        
        print("Laser position subscriber created")
        print("  Topic: /laser_pos")
        print("  Format: 'x,y' or 'none'")
        print()
        
        # Create timer for control loop.
        # Lower rate (15Hz) gives camera time to detect laser position changes.
        # If servos move too fast, the laser detector can't track it between frames.
        try:
            control_hz = float(os.environ.get("TRACKER_CONTROL_HZ", "15.0"))
        except ValueError:
            control_hz = 15.0
        control_hz = max(2.0, min(60.0, control_hz))
        self.control_period_s = 1.0 / control_hz
        self.timer = self.create_timer(self.control_period_s, self.control_loop)
        
        print("Tracker node initialized")
        print("Waiting for ball detection data...")
        print("(Press Ctrl+C to stop)")
        print()

        # Log effective config for easier tuning
        print()
        print("=" * 60)
        print("TRACKER CONFIGURATION (SLOW MODE for Camera Tracking)")
        print("=" * 60)
        print("NOTE: Servos move SLOWLY so camera can track laser position")
        print("      between frames. Fast movements cause tracking loss.")
        print("=" * 60)
        print(f"Control Loop:")
        print(f"  Rate: {1.0 / self.control_period_s:.1f} Hz")
        print(f"  Image: {self.image_width}x{self.image_height} px")
        print()
        print(f"Detection Filters:")
        print(f"  Max jump: {self.max_jump_px_s:.0f} px/s (rejects glitches)")
        print(f"  Timeout: {self.timeout}s (resets to center if no ball)")
        print()
        print(f"Laser Feedback:")
        print(f"  Timeout: {self.laser_timeout}s (falls back to estimation)")
        print(f"  Max estimation steps: {self.laser_estimated_max_steps} (before recentering)")
        print()
        print(f"Camera Model:")
        print(f"  HFOV/VFOV: {self.controller.hfov_deg}/{self.controller.vfov_deg}°")
        if self.controller.fx_px > 0 or self.controller.fy_px > 0:
            print(f"  Focal length: fx={self.controller.fx_px:.1f}, fy={self.controller.fy_px:.1f} px")
        print()
        print(f"PID Controller (Initial):")
        print(f"  Kp: {self.controller.kp_pan}/{self.controller.kp_tilt} (proportional)")
        print(f"  Kd: {self.controller.kd_pan}/{self.controller.kd_tilt} (derivative)")
        print(f"  Ki: {self.controller._ki_pan}/{self.controller._ki_tilt} (integral)")
        print(f"  Adaptive: {'ENABLED' if self.controller._adaptive_enabled else 'DISABLED'}")
        if self.controller._adaptive_enabled:
            print(f"    - Auto-tunes every {self.controller._adjustment_interval}s")
            print(f"    - Kp range: 0.3 to 2.0")
            print(f"    - Detects oscillations & adjusts gains")
            print(f"    - Enables Ki if stuck (max: 0.2)")
        print()
        print(f"Servo Configuration:")
        print(f"  Center: pan={self.controller.pan_center}°, tilt={self.controller.tilt_center}°")
        print(f"  Invert: pan={self.controller.invert_pan}, tilt={self.controller.invert_tilt}")
        print(f"  Limits: pan=[{self.controller.pan_min}°, {self.controller.pan_max}°], tilt=[{self.controller.tilt_min}°, {self.controller.tilt_max}°]")
        print(f"  Max speed: {self.controller.max_speed_deg_s}°/s")
        print(f"  Max step: {self.controller.max_step_deg}° per tick")
        print()
        print(f"Smoothing & Filtering:")
        print(f"  Ball position EMA: α={self.controller.ema_alpha}")
        print(f"  Deadband: {self.controller.deadband_deg}° (prevents buzzing)")
        print(f"  Max dt: {self.controller.max_dt_s}s (rate limiter clamp)")
        print("=" * 60)
        print()
    
    def coords_callback(self, msg: String) -> None:
        """Callback for ball coordinates messages"""
        parsed = parse_coordinates(msg.data)
        self.coords = parsed
        self.last_coords_update = time.time()
    
    def laser_callback(self, msg: String) -> None:
        """Callback for laser position messages"""
        parsed = parse_laser_position(msg.data)
        self.laser_pos = parsed
        self.last_laser_update = time.time()

    def _laser_in_frame(self, x: float, y: float) -> bool:
        return (0.0 <= x < float(self.image_width)) and (0.0 <= y < float(self.image_height))
    
    def control_loop(self) -> None:
        """Main control loop - runs at `TRACKER_CONTROL_HZ`."""
        current_time = time.time()
        
        # Check for ball detection coordinates
        if self.coords is not None:
            # Check if coordinates are stale
            if current_time - self.last_coords_update < self.timeout:
                x1, y1, x2, y2 = self.coords
                
                # Calculate ball center for laser position
                ball_center_x = (x1 + x2) / 2.0
                ball_center_y = (y1 + y2) / 2.0
                ball_width = abs(x2 - x1)
                ball_height = abs(y2 - y1)

                # Check if ball is clipped at image edge (partial detection)
                # RoboFlow often returns bbox extending to image edge (e.g., x2=640.0)
                is_clipped_left = x1 <= 1.0
                is_clipped_right = x2 >= (self.image_width - 1.0)
                is_clipped_top = y1 <= 1.0
                is_clipped_bottom = y2 >= (self.image_height - 1.0)
                is_clipped = is_clipped_left or is_clipped_right or is_clipped_top or is_clipped_bottom
                
                # Clamp to image bounds to keep math sane if detector returns out-of-range coords.
                ball_center_x = max(0.0, min(float(self.image_width - 1), ball_center_x))
                ball_center_y = max(0.0, min(float(self.image_height - 1), ball_center_y))
                
                # Log clipped detections occasionally for debugging
                if is_clipped and self._log_counter % 10 == 0:
                    edges = []
                    if is_clipped_left: edges.append("LEFT")
                    if is_clipped_right: edges.append("RIGHT")
                    if is_clipped_top: edges.append("TOP")
                    if is_clipped_bottom: edges.append("BOTTOM")
                    self.get_logger().debug(f"Ball clipped at edge: {', '.join(edges)}")

                # Glitch filter: reject single-frame jumps that exceed max_jump_px_s.
                if self._last_ball_center is not None and self._last_ball_time_s is not None:
                    dt = max(0.001, current_time - self._last_ball_time_s)
                    dx = abs(ball_center_x - self._last_ball_center[0])
                    dy = abs(ball_center_y - self._last_ball_center[1])
                    if (dx / dt) > self.max_jump_px_s or (dy / dt) > self.max_jump_px_s:
                        if not hasattr(self, "_jump_warn_ctr"):
                            self._jump_warn_ctr = 0
                        self._jump_warn_ctr += 1
                        if self._jump_warn_ctr % 10 == 0:
                            self.get_logger().warn(
                                f"Rejected detector jump: dx={dx:.1f}px dy={dy:.1f}px dt={dt:.3f}s "
                                f"(limit {self.max_jump_px_s:.0f}px/s)"
                            )
                        return

                self._last_ball_center = (ball_center_x, ball_center_y)
                self._last_ball_time_s = current_time
                
                # Get laser position from ROS topic (from calibration node)
                # If `/laser_pos` is not available, we fall back to open-loop:
                # keep the latest laser estimate and dead-reckon based on the servo commands.
                laser_valid = False
                laser_x: Optional[float]
                laser_y: Optional[float]
                if self.laser_pos is not None and (current_time - self.last_laser_update) < self.laser_timeout:
                    lx_meas, ly_meas = self.laser_pos
                    if self._laser_in_frame(lx_meas, ly_meas):
                        laser_valid = True
                        laser_x, laser_y = lx_meas, ly_meas
                        laser_source = "measured"
                    else:
                        # Treat out-of-range measurements as invalid (e.g., detector bug / partial frame math).
                        laser_x, laser_y = None, None
                        laser_source = "estimated"
                else:
                    # No laser measurement: let the controller use its internal estimate based on servo motion.
                    laser_x, laser_y = None, None
                    laser_source = "estimated"

                if laser_valid:
                    self._laser_estimated_steps = 0
                    self._laser_recovery_active = False
                    
                    # Diagnostic: detect if servos are moving but laser isn't
                    if not hasattr(self, "_prev_servo_pos"):
                        self._prev_servo_pos = (self.controller.pan_angle, self.controller.tilt_angle)
                        self._prev_laser_pos = (laser_x, laser_y)
                        self._stationary_laser_counter = 0
                        self._laser_move_log_counter = 0
                    else:
                        prev_pan, prev_tilt = self._prev_servo_pos
                        prev_lx, prev_ly = self._prev_laser_pos
                        
                        servo_moved = abs(self.controller.pan_angle - prev_pan) > 2.0 or abs(self.controller.tilt_angle - prev_tilt) > 2.0
                        laser_moved = abs(laser_x - prev_lx) > 5.0 or abs(laser_y - prev_ly) > 5.0
                        
                        # Log when laser actually moves
                        if laser_moved:
                            self._laser_move_log_counter += 1
                            if self._laser_move_log_counter % 3 == 0:
                                dlx = laser_x - prev_lx
                                dly = laser_y - prev_ly
                                self.get_logger().info(
                                    f"Laser moved: ({prev_lx:.0f},{prev_ly:.0f}) -> ({laser_x:.0f},{laser_y:.0f}) "
                                    f"Δ({dlx:+.0f},{dly:+.0f}px) | "
                                    f"Servo: pan={self.controller.pan_angle:.0f}° tilt={self.controller.tilt_angle:.0f}°"
                                )
                        
                        if servo_moved and not laser_moved:
                            self._stationary_laser_counter += 1
                            if self._stationary_laser_counter >= 3:
                                if not hasattr(self, "_last_stationary_warn_s"):
                                    self._last_stationary_warn_s = 0.0
                                if (current_time - self._last_stationary_warn_s) >= 3.0:
                                    self.get_logger().error(
                                        f"SERVO MOVEMENT NOT AFFECTING LASER POSITION! "
                                        f"Pan moved {prev_pan:.0f}° -> {self.controller.pan_angle:.0f}°, "
                                        f"Tilt moved {prev_tilt:.0f}° -> {self.controller.tilt_angle:.0f}°, "
                                        f"but laser stayed at ({laser_x:.0f}, {laser_y:.0f}). "
                                        f"Check: (1) Is camera mounted on servos? (2) Are servos physically connected to laser mount?"
                                    )
                                    self._last_stationary_warn_s = current_time
                                    self._stationary_laser_counter = 0
                        elif laser_moved:
                            self._stationary_laser_counter = 0
                        
                        self._prev_servo_pos = (self.controller.pan_angle, self.controller.tilt_angle)
                        self._prev_laser_pos = (laser_x, laser_y)
                else:
                    self._laser_estimated_steps += 1
                
                # Laser lost watchdog: estimation should only last a few frames.
                # If we exceed the limit, assume the laser is in a weird place and reset.
                estimated_steps = int(self._laser_estimated_steps)

                if estimated_steps >= self.laser_estimated_max_steps:
                    self._laser_recovery_active = True
                    self._laser_estimated_steps = 0
                    self.controller.reset_to_center()
                    self.controller.reset_laser_estimate_to_center()
                    if self.servo is not None:
                        self.servo.set_pan(self.controller.pan_center)
                        self.servo.set_tilt(self.controller.tilt_center)

                    # Throttle recovery logs (avoid spamming at control rate).
                    if (current_time - self._last_laser_recovery_log_s) >= 1.0:
                        self.get_logger().warn(
                            f"Estimation exceeded {self.laser_estimated_max_steps} frames; recentering"
                        )
                        self._last_laser_recovery_log_s = current_time
                    return

                pan_angle, tilt_angle = self.controller.update(
                    ball_center_x=ball_center_x,
                    ball_center_y=ball_center_y,
                    ball_width=ball_width,
                    ball_height=ball_height,
                    laser_x=laser_x,
                    laser_y=laser_y,
                    now_s=current_time,
                )
                
                if self.servo is not None:
                    self.servo.set_pan(pan_angle)
                    self.servo.set_tilt(tilt_angle)

                # Saturation warnings: if a servo hits its limit for a while, the inversion
                # setting might be wrong for the current physical setup.
                
                # Pan saturation check
                if not hasattr(self, "_pan_sat_since_s"):
                    self._pan_sat_since_s = None
                at_pan_min = abs(pan_angle - self.controller.pan_min) < 1e-6
                at_pan_max = abs(pan_angle - self.controller.pan_max) < 1e-6
                if at_pan_min or at_pan_max:
                    if self._pan_sat_since_s is None:
                        self._pan_sat_since_s = current_time
                    elif (current_time - self._pan_sat_since_s) >= 1.0:
                        if not hasattr(self, "_last_pan_sat_log_s"):
                            self._last_pan_sat_log_s = 0.0
                        if (current_time - self._last_pan_sat_log_s) >= 2.0:
                            self.get_logger().warn(
                                "Pan servo saturated at limit (0° or 180°). "
                                "If the laser moves the wrong way horizontally, toggle TRACKER_INVERT_PAN (0/1)."
                            )
                            self._last_pan_sat_log_s = current_time
                else:
                    self._pan_sat_since_s = None
                
                # Tilt saturation check
                if not hasattr(self, "_tilt_sat_since_s"):
                    self._tilt_sat_since_s = None
                at_tilt_min = abs(tilt_angle - self.controller.tilt_min) < 1e-6
                at_tilt_max = abs(tilt_angle - self.controller.tilt_max) < 1e-6
                if at_tilt_min or at_tilt_max:
                    if self._tilt_sat_since_s is None:
                        self._tilt_sat_since_s = current_time
                    elif (current_time - self._tilt_sat_since_s) >= 1.0:
                        if not hasattr(self, "_last_tilt_sat_log_s"):
                            self._last_tilt_sat_log_s = 0.0
                        if (current_time - self._last_tilt_sat_log_s) >= 2.0:
                            self.get_logger().warn(
                                "Tilt servo saturated at limit (0° or 180°). "
                                "If the laser moves the wrong way vertically, toggle TRACKER_INVERT_TILT (0/1)."
                            )
                            self._last_tilt_sat_log_s = current_time
                else:
                    self._tilt_sat_since_s = None
                
                # Calculate error relative to actual laser position
                # Use controller's current internal estimate for logging error.
                if self.controller._laser_est_px is not None:
                    lx, ly = self.controller._laser_est_px
                else:
                    lx, ly = (self.image_width / 2.0, self.image_height / 2.0)
                error_x = ball_center_x - lx
                error_y = ball_center_y - ly
                
                # Log less frequently to reduce spam
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                # Log adaptive tuning changes
                if self.controller._adaptive_enabled and hasattr(self.controller, '_last_tune_info'):
                    tune_info = self.controller._last_tune_info
                    if tune_info.get('kp_changed', False):
                        msg_parts = [f"Auto-tuned gains: Kp={self.controller._kp_pan_adaptive:.2f}/{self.controller._kp_tilt_adaptive:.2f}"]
                        if tune_info.get('pan_osc') or tune_info.get('tilt_osc'):
                            msg_parts.append("(reducing - oscillation detected)")
                        elif tune_info.get('avg_error', 0) > 3.0:
                            msg_parts.append("(increasing - slow convergence)")
                        self.get_logger().info(' '.join(msg_parts))
                        self.controller._last_tune_info['kp_changed'] = False
                
                if self._log_counter % 5 == 0:  # Log every 5th update to reduce spam
                    # Calculate tracking quality metrics
                    error_magnitude = math.sqrt(error_x**2 + error_y**2)
                    
                    # Track error trend (getting better or worse)
                    if not hasattr(self, '_prev_error_mag'):
                        self._prev_error_mag = error_magnitude
                        error_trend = ""
                    else:
                        error_delta = error_magnitude - self._prev_error_mag
                        if abs(error_delta) < 2.0:
                            error_trend = " (stable)"
                        elif error_delta < 0:
                            error_trend = f" (BETTER {error_delta:+.1f})"
                        else:
                            error_trend = f" (WORSE {error_delta:+.1f})"
                        self._prev_error_mag = error_magnitude
                    
                    # Show servo movement delta
                    if not hasattr(self, '_prev_pan_log'):
                        self._prev_pan_log = pan_angle
                        self._prev_tilt_log = tilt_angle
                        servo_delta = ""
                    else:
                        dpan = pan_angle - self._prev_pan_log
                        dtilt = tilt_angle - self._prev_tilt_log
                        if abs(dpan) > 0.5 or abs(dtilt) > 0.5:
                            servo_delta = f" Δ({dpan:+.1f}°,{dtilt:+.1f}°)"
                        else:
                            servo_delta = " (no move)"
                        self._prev_pan_log = pan_angle
                        self._prev_tilt_log = tilt_angle
                    
                    # Include adaptive gain info if enabled
                    if self.controller._adaptive_enabled:
                        self.get_logger().info(
                            f'Ball: ({ball_center_x:.0f},{ball_center_y:.0f}), '
                            f'Laser[{laser_source}]: ({lx:.0f},{ly:.0f}), '
                            f'error: {error_magnitude:.1f}px{error_trend} ({error_x:+.0f},{error_y:+.0f}), '
                            f'servo: pan={pan_angle:.1f}° tilt={tilt_angle:.1f}°{servo_delta} | '
                            f'Kp:{self.controller._kp_pan_adaptive:.2f}/{self.controller._kp_tilt_adaptive:.2f} '
                            f'Ki:{self.controller._ki_pan:.3f}/{self.controller._ki_tilt:.3f}'
                        )
                    else:
                        self.get_logger().info(
                            f'Ball: ({ball_center_x:.0f},{ball_center_y:.0f}), '
                            f'Laser[{laser_source}]: ({lx:.0f},{ly:.0f}), '
                            f'error: {error_magnitude:.1f}px{error_trend} ({error_x:+.0f},{error_y:+.0f}), '
                            f'servo: pan={pan_angle:.1f}° tilt={tilt_angle:.1f}°{servo_delta}'
                        )
                
                self.last_detection_time = current_time
            else:
                # Coordinates are stale, reset to center
                if current_time - self.last_detection_time > self.timeout:
                    self.controller.reset_to_center()
                    if self.servo is not None:
                        self.servo.set_pan(self.controller.pan_center)
                        self.servo.set_tilt(self.controller.tilt_center)
                    self.get_logger().info(f"No detection for {self.timeout}s, resetting to center")
                    self.last_detection_time = current_time
                
        else:
            # No coordinates available, check if we should reset
            if current_time - self.last_detection_time > self.timeout:
                self.controller.reset_to_center()
                if self.servo is not None:
                    self.servo.set_pan(self.controller.pan_center)
                    self.servo.set_tilt(self.controller.tilt_center)
                # Only log once per timeout period
                if int(current_time) % int(self.timeout + 1) == 0:
                    self.get_logger().info(f"No detection for {self.timeout}s, resetting to center")
                self.last_detection_time = current_time
            
            pass


def main(args=None):
    """Main function"""
    if not RCLPY_AVAILABLE:
        print("Error: rclpy is not available. Cannot run tracker node.")
        print("Please install ROS2 and rclpy, or use the Rust version instead.")
        sys.exit(1)
    
    rclpy.init(args=args)
    
    node = BallTrackerNode()
    
    # Handle shutdown gracefully
    def signal_handler(sig, frame):
        print("\nShutting down tracker node...")
        if node.servo is not None:
            node.servo.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.servo is not None:
            node.servo.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
