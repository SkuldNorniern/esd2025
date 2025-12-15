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
    """Direct position-to-angle mapping controller for ball tracking (no laser feedback)."""

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

        # Direct mapping gain: how much to scale pixel offset to servo angle
        # Lower values = less aggressive, higher = more aggressive
        self.position_gain = self._env_float("TRACKER_POSITION_GAIN", 0.8)
        
        # Smoothing for servo commands to avoid jitter
        self.command_smoothing = self._env_float("TRACKER_COMMAND_SMOOTHING", 0.3)

        # Servo center angles (image center maps to these angles)
        self.pan_center = self._env_float("TRACKER_PAN_CENTER_DEG", 90.0)
        self.tilt_center = self._env_float("TRACKER_TILT_CENTER_DEG", 90.0)

        # Current servo positions (degrees) - smoothed commands
        self.pan_angle = float(self.pan_center)
        self.tilt_angle = float(self.tilt_center)

        # Servo limits (degrees)
        self.pan_min = self._env_float("TRACKER_PAN_MIN_DEG", 0.0)
        self.pan_max = self._env_float("TRACKER_PAN_MAX_DEG", 180.0)
        self.tilt_min = self._env_float("TRACKER_TILT_MIN_DEG", 0.0)
        self.tilt_max = self._env_float("TRACKER_TILT_MAX_DEG", 180.0)

        # Invert servo direction if needed
        self.invert_pan = self._env_bool("TRACKER_INVERT_PAN", False)
        self.invert_tilt = self._env_bool("TRACKER_INVERT_TILT", False)

        # Optional EMA smoothing of ball position input (0..1)
        self.ball_smoothing = self._env_float("TRACKER_BALL_SMOOTHING", 0.4)
        self._ball_ema: Optional[Tuple[float, float]] = None

        self.last_update = time.time()

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
        a = max(0.0, min(1.0, self.ball_smoothing))
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
        """
        Update servo positions based on ball position using direct position-to-angle mapping.
        
        The approach:
        - Image center (cx_px, cy_px) maps to servo center angles (pan_center, tilt_center)
        - Ball offset from image center is converted to angular offset
        - Servo angle = center_angle + angular_offset * position_gain
        """

        # Smooth ball position (helps noisy detections).
        self._ball_ema = self._ema(self._ball_ema, ball_center_x, ball_center_y)
        ball_center_x, ball_center_y = self._ball_ema

        # Calculate pixel offset from image center
        offset_x = ball_center_x - self.cx_px
        offset_y = ball_center_y - self.cy_px

        # Convert pixel offset to angular offset (degrees)
        angle_offset_x = self._px_to_deg_x(offset_x)
        angle_offset_y = self._px_to_deg_y(offset_y)

        # Calculate target servo angles
        # Center of image (cx_px, cy_px) corresponds to (pan_center, tilt_center)
        # Positive offset_x (ball right of center) should move pan servo accordingly
        target_pan = self.pan_center + (angle_offset_x * self.position_gain)
        target_tilt = self.tilt_center + (angle_offset_y * self.position_gain)

        # Apply inversion if needed
        if self.invert_pan:
            target_pan = self.pan_center - (angle_offset_x * self.position_gain)
        if self.invert_tilt:
            target_tilt = self.tilt_center - (angle_offset_y * self.position_gain)

        # Clamp to servo limits
        target_pan = max(self.pan_min, min(self.pan_max, target_pan))
        target_tilt = max(self.tilt_min, min(self.tilt_max, target_tilt))

        # Smooth the command to avoid jitter
        alpha = max(0.0, min(1.0, self.command_smoothing))
        self.pan_angle = self.pan_angle * (1.0 - alpha) + target_pan * alpha
        self.tilt_angle = self.tilt_angle * (1.0 - alpha) + target_tilt * alpha

        self.last_update = now_s

        return (self.pan_angle, self.tilt_angle)
    
    def reset_to_center(self) -> None:
        """Reset to center position"""
        self.pan_angle = float(self.pan_center)
        self.tilt_angle = float(self.tilt_center)
        self._ball_ema = None


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
        
        # Image dimensions (should match camera resolution: 640x640)
        # Updated from 320x240 to match cam_test resolution change
        self.image_width = 640
        self.image_height = 640
        self.controller = Controller(self.image_width, self.image_height)

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
        print("NOTE: This tracker does NOT use laser feedback.")
        print("  It directly maps ball position to servo angles.")
        print()
        
        # Create timer for control loop.
        # Higher rates reduce lag, but if your servos buzz or overshoot, reduce TRACKER_CONTROL_HZ.
        # Increased default to 30Hz for better responsiveness
        try:
            control_hz = float(os.environ.get("TRACKER_CONTROL_HZ", "30.0"))
        except ValueError:
            control_hz = 30.0
        control_hz = max(2.0, min(60.0, control_hz))
        self.control_period_s = 1.0 / control_hz
        self.timer = self.create_timer(self.control_period_s, self.control_loop)
        
        print("Tracker node initialized")
        print("Waiting for ball detection data...")
        print("(Press Ctrl+C to stop)")
        print()

        # Log effective config for easier tuning
        print("Tracker config (direct position mapping - no laser feedback):")
        print(f"  control_hz: {1.0 / self.control_period_s:.1f}")
        print(f"  image: {self.image_width}x{self.image_height}")
        print(f"  image_center: ({self.controller.cx_px:.1f}, {self.controller.cy_px:.1f}) px")
        print(f"  servo_center: ({self.controller.pan_center:.1f}, {self.controller.tilt_center:.1f}) deg")
        print(f"  servo_limits: pan [{self.controller.pan_min:.0f}, {self.controller.pan_max:.0f}], tilt [{self.controller.tilt_min:.0f}, {self.controller.tilt_max:.0f}] deg")
        print(f"  max_jump_px_s: {self.max_jump_px_s}")
        print(f"  HFOV/VFOV deg: {self.controller.hfov_deg}/{self.controller.vfov_deg}")
        print(f"  fx/fy px: {self.controller.fx_px}/{self.controller.fy_px}")
        print(f"  position_gain: {self.controller.position_gain} (TRACKER_POSITION_GAIN env var)")
        print(f"  command_smoothing: {self.controller.command_smoothing} (TRACKER_COMMAND_SMOOTHING env var)")
        print(f"  ball_smoothing: {self.controller.ball_smoothing} (TRACKER_BALL_SMOOTHING env var)")
        print(f"  invert pan/tilt: {self.controller.invert_pan}/{self.controller.invert_tilt}")
        print()
        print("How it works:")
        print("  - Ball at image center -> servo at center angles")
        print("  - Ball offset from center -> proportional servo angle offset")
        print("  - No laser feedback required (open-loop control)")
        print()
    
    def coords_callback(self, msg: String) -> None:
        """Callback for ball coordinates messages"""
        parsed = parse_coordinates(msg.data)
        self.coords = parsed
        self.last_coords_update = time.time()
    
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
                
                # Update servo positions based on ball position
                # No laser feedback - direct position-to-angle mapping
                pan_angle, tilt_angle = self.controller.update(
                    ball_center_x=ball_center_x,
                    ball_center_y=ball_center_y,
                    ball_width=ball_width,
                    ball_height=ball_height,
                    laser_x=None,  # Not used in this controller
                    laser_y=None,  # Not used in this controller
                    now_s=current_time,
                )
                
                if self.servo is not None:
                    self.servo.set_pan(pan_angle)
                    self.servo.set_tilt(tilt_angle)

                # If we are saturating the tilt at 0°/180° for a while, it often means:
                # - the mechanical range is insufficient, OR
                # - `TRACKER_INVERT_TILT` is wrong for the current servo installation.
                # This doesn't auto-flip the sign (too risky), but it gives a clear hint.
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
                
                # Calculate offset from image center
                offset_x = ball_center_x - self.controller.cx_px
                offset_y = ball_center_y - self.controller.cy_px
                
                # Log less frequently to reduce spam
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 5 == 0:  # Log every 5th update
                    self.get_logger().info(
                        f'Ball: ({ball_center_x:.1f}, {ball_center_y:.1f}) px, '
                        f'offset from center: ({offset_x:+.1f}, {offset_y:+.1f}) px, '
                        f'servo: pan={pan_angle:.1f}°, tilt={tilt_angle:.1f}°'
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
