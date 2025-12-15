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
        self.set_pan(90.0)
        self.set_tilt(90.0)
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
        self.kp_pan = self._env_float("TRACKER_KP_PAN", 0.8)
        self.kp_tilt = self._env_float("TRACKER_KP_TILT", 0.8)
        self.kd_pan = self._env_float("TRACKER_KD_PAN", 0.0)
        self.kd_tilt = self._env_float("TRACKER_KD_TILT", 0.0)

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

        # Servo speed limit (deg/sec)
        self.max_speed_deg_s = self._env_float("TRACKER_MAX_SPEED_DEG_S", 120.0)

        # Small deadband in angular domain to prevent buzzing
        self.deadband_deg = self._env_float("TRACKER_DEADBAND_DEG", 0.4)

        # Optional EMA smoothing of input points (0..1)
        self.ema_alpha = self._env_float("TRACKER_EMA_ALPHA", 0.4)

        self.last_update = time.time()
        self.last_err_pan_deg = 0.0
        self.last_err_tilt_deg = 0.0
        self._ball_ema: Optional[Tuple[float, float]] = None
        self._laser_ema: Optional[Tuple[float, float]] = None

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

    def update(
        self,
        ball_center_x: float,
        ball_center_y: float,
        ball_width: float,
        ball_height: float,
        laser_x: float,
        laser_y: float,
        now_s: float,
    ) -> Tuple[float, float]:
        """Update servo positions based on ball and laser positions in pixels."""

        # Smooth inputs (helps noisy detections / laser dot jitter).
        self._ball_ema = self._ema(self._ball_ema, ball_center_x, ball_center_y)
        self._laser_ema = self._ema(self._laser_ema, laser_x, laser_y)
        ball_center_x, ball_center_y = self._ball_ema
        laser_x, laser_y = self._laser_ema

        # If laser is already within the ball bbox, hold position (no hunting).
        if abs(ball_center_x - laser_x) <= (ball_width / 2.0) and abs(ball_center_y - laser_y) <= (ball_height / 2.0):
            self.last_update = now_s
            self.last_err_pan_deg = 0.0
            self.last_err_tilt_deg = 0.0
            return (self.pan_angle, self.tilt_angle)

        dt = max(0.001, now_s - self.last_update)

        err_px_x = ball_center_x - laser_x
        err_px_y = ball_center_y - laser_y

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

        delta_pan = self.kp_pan * err_pan_deg + self.kd_pan * derr_pan
        delta_tilt = self.kp_tilt * err_tilt_deg + self.kd_tilt * derr_tilt

        if self.invert_pan:
            delta_pan = -delta_pan
        if self.invert_tilt:
            delta_tilt = -delta_tilt

        # Rate limit (deg/sec)
        max_delta = self.max_speed_deg_s * dt
        delta_pan = max(-max_delta, min(max_delta, delta_pan))
        delta_tilt = max(-max_delta, min(max_delta, delta_tilt))

        self.pan_angle = max(self.pan_min, min(self.pan_max, self.pan_angle + delta_pan))
        self.tilt_angle = max(self.tilt_min, min(self.tilt_max, self.tilt_angle + delta_tilt))

        self.last_update = now_s
        self.last_err_pan_deg = err_pan_deg
        self.last_err_tilt_deg = err_tilt_deg

        return (self.pan_angle, self.tilt_angle)
    
    def reset_to_center(self) -> None:
        """Reset to center position"""
        self.pan_angle = float(self.pan_center)
        self.tilt_angle = float(self.tilt_center)


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
        
        # Image dimensions (should match camera resolution: 640x640)
        # Updated from 320x240 to match cam_test resolution change
        self.image_width = 640
        self.image_height = 640
        self.controller = Controller(self.image_width, self.image_height)

        # Reject one-frame detector glitches that jump too far (px/sec).
        # This is extremely common with bounding-box detectors and causes servo snaps.
        try:
            self.max_jump_px_s = float(os.environ.get("TRACKER_MAX_JUMP_PX_S", "2000.0"))
        except ValueError:
            self.max_jump_px_s = 2000.0
        self._last_ball_center: Optional[Tuple[float, float]] = None
        self._last_ball_time_s: Optional[float] = None
        
        # Ball coordinates state
        self.coords: Optional[Tuple[float, float, float, float]] = None
        self.last_coords_update = time.time()
        
        # Laser position state (from ROS topic)
        self.laser_pos: Optional[Tuple[float, float]] = None
        self.last_laser_update = time.time()
        self.laser_timeout = 1.0  # If no laser update for 1s, fall back to open-loop (assume center)
        
        # Timeout for reset (2 seconds)
        self.timeout = 2.0
        self.last_detection_time = time.time()
        
        # Create subscriber for ball coordinates
        print("Subscribing to ball detection topic...")
        self.subscription = self.create_subscription(
            String,
            '/ball_coords',
            self.coords_callback,
            10
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
            10
        )
        
        print("Laser position subscriber created")
        print("  Topic: /laser_pos")
        print("  Format: 'x,y' or 'none'")
        print()
        
        # Create timer for control loop.
        # Higher rates reduce lag, but if your servos buzz or overshoot, reduce TRACKER_CONTROL_HZ.
        try:
            control_hz = float(os.environ.get("TRACKER_CONTROL_HZ", "20.0"))
        except ValueError:
            control_hz = 20.0
        control_hz = max(2.0, min(60.0, control_hz))
        self.control_period_s = 1.0 / control_hz
        self.timer = self.create_timer(self.control_period_s, self.control_loop)
        
        print("Tracker node initialized")
        print("Waiting for ball detection data...")
        print("(Press Ctrl+C to stop)")
        print()

        # Log effective config for easier tuning
        print("Tracker config:")
        print(f"  control_hz: {1.0 / self.control_period_s:.1f}")
        print(f"  image: {self.image_width}x{self.image_height}")
        print(f"  laser_timeout_s: {self.laser_timeout}")
        print(f"  max_jump_px_s: {self.max_jump_px_s}")
        print(f"  HFOV/VFOV deg: {self.controller.hfov_deg}/{self.controller.vfov_deg}")
        print(f"  fx/fy px: {self.controller.fx_px}/{self.controller.fy_px}")
        print(f"  kp pan/tilt: {self.controller.kp_pan}/{self.controller.kp_tilt}")
        print(f"  kd pan/tilt: {self.controller.kd_pan}/{self.controller.kd_tilt}")
        print(f"  invert pan/tilt: {self.controller.invert_pan}/{self.controller.invert_tilt}")
        print(f"  center pan/tilt deg: {self.controller.pan_center}/{self.controller.tilt_center}")
        print(f"  max_speed_deg_s: {self.controller.max_speed_deg_s}")
        print(f"  deadband_deg: {self.controller.deadband_deg}")
        print(f"  ema_alpha: {self.controller.ema_alpha}")
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
    
    def control_loop(self) -> None:
        """Main control loop - runs at 10Hz"""
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

                # Clamp to image bounds to keep math sane if detector returns out-of-range coords.
                ball_center_x = max(0.0, min(float(self.image_width - 1), ball_center_x))
                ball_center_y = max(0.0, min(float(self.image_height - 1), ball_center_y))

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
                # If `/laser_pos` is not available, we fall back to open-loop: assume the laser is at image center.
                # This will still "track to center" (camera-axis tracking), but accurate laser pointing needs real `/laser_pos`.
                if self.laser_pos is not None and (current_time - self.last_laser_update) < self.laser_timeout:
                    laser_x, laser_y = self.laser_pos
                    laser_source = "measured"
                else:
                    laser_x = self.image_width / 2.0
                    laser_y = self.image_height / 2.0
                    laser_source = "assumed_center"
                
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
                
                # Calculate error relative to actual laser position
                error_x = ball_center_x - laser_x
                error_y = ball_center_y - laser_y
                
                # Log less frequently to reduce spam
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 3 == 0:  # Log every 3rd update for better debugging
                    self.get_logger().info(
                        f'Ball: ({ball_center_x:.1f}, {ball_center_y:.1f}), '
                        f'Laser[{laser_source}]: ({laser_x:.1f}, {laser_y:.1f}), '
                        f'err: ({error_x:.1f}, {error_y:.1f}), '
                        f'pan={pan_angle:.1f}°, tilt={tilt_angle:.1f}°'
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
