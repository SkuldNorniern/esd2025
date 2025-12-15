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

import time
import os
from typing import Optional, Tuple
import signal
import sys

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
        
        # Set initial position to center (90 degrees)
        self.set_pan(90.0)
        self.set_tilt(90.0)
    
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
    """PID controller (P-only) for ball tracking"""
    
    def __init__(self, image_width: int, image_height: int):
        # Proportional gains - higher for accurate pointing
        self.kp_pan = 0.12  # Increased for faster tracking
        self.kp_tilt = 0.12  # Increased for faster tracking
        
        # Current servo positions (degrees)
        self.pan_angle = 90.0  # Start at center
        self.tilt_angle = 90.0
        
        # Image dimensions (for coordinate conversion)
        self.image_width = image_width
        self.image_height = image_height
        
        # Servo limits (degrees)
        self.pan_min = 0.0
        self.pan_max = 180.0
        self.tilt_min = 0.0
        self.tilt_max = 180.0
        
        # Rate limiting (max change per update)
        self.max_rate = 5.0  # Max 5 degrees per update for faster tracking
        
        self.last_update = time.time()
        
        # Track previous ball position to detect if ball is actually moving
        self.last_ball_center_x: Optional[float] = None
        self.last_ball_center_y: Optional[float] = None
        self.position_change_threshold = 5.0  # Pixels - if ball moves less than this, consider it stationary (increased to prevent overshooting)
    
    def update(self, x1: float, y1: float, x2: float, y2: float, 
               laser_x: Optional[float] = None, laser_y: Optional[float] = None) -> Tuple[float, float]:
        """Update servo positions based on ball detection"""
        # Calculate ball center
        ball_center_x = (x1 + x2) / 2.0
        ball_center_y = (y1 + y2) / 2.0
        
        # Calculate ball area for adaptive deadzone
        ball_width = abs(x2 - x1)
        ball_height = abs(y2 - y1)
        ball_area = ball_width * ball_height
        
        # Use provided laser position or fallback to image center
        if laser_x is not None and laser_y is not None:
            target_x = laser_x
            target_y = laser_y
        else:
            # Fallback to image center
            target_x = self.image_width / 2.0
            target_y = self.image_height / 2.0
        
        # Calculate error relative to laser position
        error_x = ball_center_x - target_x
        error_y = ball_center_y - target_y
        error_magnitude = (error_x**2 + error_y**2)**0.5
        
        # Check if laser is already within the ball's area
        # If the error is less than half the ball size, the laser is likely on the ball
        ball_half_width = ball_width / 2.0
        ball_half_height = ball_height / 2.0
        if abs(error_x) < ball_half_width and abs(error_y) < ball_half_height:
            # Laser is within the ball's bounding box - no need to move
            return (self.pan_angle, self.tilt_angle)
        
        # Store image center for normalization (used later)
        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0
        
        # Check if ball position actually changed significantly
        # But allow movement if error is large (ball is far from center)
        if self.last_ball_center_x is not None and self.last_ball_center_y is not None:
            ball_moved_x = abs(ball_center_x - self.last_ball_center_x)
            ball_moved_y = abs(ball_center_y - self.last_ball_center_y)
            
            # If ball hasn't moved much AND error is small, don't update servos to prevent overshooting
            # This helps when the ball is stationary and we're already close to it
            if (ball_moved_x < self.position_change_threshold and 
                ball_moved_y < self.position_change_threshold and
                error_magnitude < 30.0):  # Reduced threshold to prevent overshooting
                # Ball is stationary and we're close to center, keep current servo positions
                return (self.pan_angle, self.tilt_angle)
        
        # Update last known ball position
        self.last_ball_center_x = ball_center_x
        self.last_ball_center_y = ball_center_y
        
        # Normalize error to -1.0 to 1.0 range (use image center for normalization)
        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0
        normalized_error_x = error_x / image_center_x
        normalized_error_y = error_y / image_center_y
        
        # Calculate direct angle correction based on error
        # Map image error to servo angle change
        # For accurate pointing, we want direct proportional control
        
        # Pan control:
        # Image center: (320, 320) for 640x640
        # Servo center: 90° (both pan and tilt)
        # error_x > 0: ball is right of center (x > 320) -> pan right (increase angle toward 180°)
        # error_x < 0: ball is left of center (x < 320) -> pan left (decrease angle toward 0°)
        # Direct proportional control
        if normalized_error_x > 0:
            # Ball is on the right - reduce movement by 5%
            delta_pan = self.kp_pan * normalized_error_x * 90.0 * 0.95  # 5% less movement
        elif normalized_error_x < 0:
            # Ball is on the left - full movement
            delta_pan = self.kp_pan * normalized_error_x * 90.0
        else:
            delta_pan = 0.0
        
        # Tilt control:
        # Image center: (320, 320) for 640x640
        # Servo center: 90° (both pan and tilt)
        # error_y > 0: ball is below laser (y > laser_y) -> tilt down (increase angle toward 180°)
        # error_y < 0: ball is above laser (y < laser_y) -> tilt up (decrease angle toward 0°)
        # Image coordinates: (0,0) is top-left, y increases downward
        # gpiozero Servo mapping: -1.0 = 0° (up), 0.0 = 90° (center), 1.0 = 180° (down)
        # Logs show tilt direction is wrong - when ball is above, tilt goes down
        # Need to invert: when error_y < 0 (ball above), we want tilt up (decrease angle)
        # Inverted: delta_tilt = -kp * normalized_error_y
        #   When error_y < 0: delta_tilt = -kp * (negative) = positive -> angle increases -> tilt down (WRONG!)
        # Wait, that's still wrong. Let me check the logs more carefully.
        # From logs: Ball above (error_y < 0), tilt going down (angle > 90) - WRONG
        # So we need: when error_y < 0, delta_tilt should be negative to decrease angle
        # Direct: delta_tilt = kp * normalized_error_y -> when error_y < 0, delta_tilt < 0 -> correct!
        # But logs show it's wrong, so try inverting
        delta_tilt = -self.kp_tilt * normalized_error_y * 90.0  # Inverted to fix direction
        
        # Adaptive dead zone based on ball size and movement
        # Larger ball = larger dead zone (ball is closer, less movement needed)
        # Smaller ball = smaller dead zone (ball is farther, more movement needed)
        # Also consider if ball is actually moving
        base_dead_zone = 0.03  # 3% of image size (about 19 pixels for 640x640)
        max_dead_zone = 0.12  # 12% of image size (about 77 pixels for 640x640)
        
        # Normalize ball area (assume max ball area is about 20% of image = 0.2 * 640 * 640 = 81920)
        max_ball_area = 81920.0
        area_factor = min(ball_area / max_ball_area, 1.0)  # Clamp to 1.0
        dead_zone = base_dead_zone + (max_dead_zone - base_dead_zone) * area_factor
        
        # Check if ball is actually moving
        ball_moving = True
        if self.last_ball_center_x is not None and self.last_ball_center_y is not None:
            ball_moved_x = abs(ball_center_x - self.last_ball_center_x)
            ball_moved_y = abs(ball_center_y - self.last_ball_center_y)
            if ball_moved_x < 2.0 and ball_moved_y < 2.0:
                # Ball is stationary - increase dead zone significantly
                dead_zone *= 2.0  # Double the dead zone when ball is stationary
        
        # Apply dead zone - but don't apply to left side to ensure it moves
        if abs(normalized_error_x) < dead_zone and normalized_error_x >= 0:
            # Only apply dead zone for right side or center, not left side
            delta_pan = 0.0
        if abs(normalized_error_y) < dead_zone:
            delta_tilt = 0.0
        
        # Rate limiting
        elapsed = time.time() - self.last_update
        max_delta = self.max_rate * max(elapsed, 0.01)  # Cap at reasonable rate
        
        delta_pan = max(-max_delta, min(max_delta, delta_pan))
        delta_tilt = max(-max_delta, min(max_delta, delta_tilt))
        
        # Update angles
        self.pan_angle += delta_pan
        self.tilt_angle += delta_tilt
        
        # Clamp to limits
        self.pan_angle = max(self.pan_min, min(self.pan_max, self.pan_angle))
        self.tilt_angle = max(self.tilt_min, min(self.tilt_max, self.tilt_angle))
        
        self.last_update = time.time()
        
        return (self.pan_angle, self.tilt_angle)
    
    def reset_to_center(self) -> None:
        """Reset to center position"""
        self.pan_angle = 90.0
        self.tilt_angle = 90.0


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
        
        # Ball coordinates state
        self.coords: Optional[Tuple[float, float, float, float]] = None
        self.last_coords_update = time.time()
        
        # Laser position state (from ROS topic)
        self.laser_pos: Optional[Tuple[float, float]] = None
        self.last_laser_update = time.time()
        self.laser_timeout = 1.0  # If no laser update for 1s, use image center as fallback
        
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
        
        # Create publisher for laser position (for real-time calibration)
        self.laser_pub = self.create_publisher(String, '/laser_pos', 10)
        print("Laser position publisher created")
        print("  Topic: /laser_pos")
        print("  Format: 'x,y' or 'none'")
        print()
        
        # Create timer for control loop (10Hz = 100ms)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        print("Tracker node initialized")
        print("Waiting for ball detection data...")
        print("(Press Ctrl+C to stop)")
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
                
                # Get laser position from ROS topic (from calibration node), or fallback to image center
                # Check if laser is within the ball's bounding box area (avoids feedback loop)
                if (self.laser_pos is not None and 
                    current_time - self.last_laser_update < self.laser_timeout):
                    laser_x, laser_y = self.laser_pos
                    # Check if laser position is within the ball's bounding box
                    ball_min_x = min(x1, x2)
                    ball_max_x = max(x1, x2)
                    ball_min_y = min(y1, y2)
                    ball_max_y = max(y1, y2)
                    if (ball_min_x <= laser_x <= ball_max_x and 
                        ball_min_y <= laser_y <= ball_max_y):
                        # Laser is within ball area - likely feedback loop, use image center instead
                        laser_x = self.image_width / 2.0
                        laser_y = self.image_height / 2.0
                else:
                    # Fallback to image center if no laser position received
                    laser_x = self.image_width / 2.0
                    laser_y = self.image_height / 2.0
                
                # Publish laser position (where servos are pointing = ball center when tracking)
                # This is for calibration - the actual laser position should come from a separate calibration node
                # that detects where the laser dot actually appears in the image
                laser_msg = String()
                laser_msg.data = f"{ball_center_x:.2f},{ball_center_y:.2f}"
                self.laser_pub.publish(laser_msg)
                
                # Update controller with laser position for accurate error calculation
                pan_angle, tilt_angle = self.controller.update(x1, y1, x2, y2, laser_x, laser_y)
                
                if self.servo is not None:
                    self.servo.set_pan(pan_angle)
                    self.servo.set_tilt(tilt_angle)
                
                # Calculate error relative to actual laser position
                error_x = ball_center_x - laser_x
                error_y = ball_center_y - laser_y
                
                # Calculate what the delta should be for debugging
                normalized_err_x = error_x / (self.image_width / 2.0)
                normalized_err_y = error_y / (self.image_height / 2.0)
                expected_delta_pan = self.controller.kp_pan * normalized_err_x * 90.0
                if normalized_err_x > 0:
                    expected_delta_pan *= 0.95  # Right side is 5% less
                expected_delta_tilt = -self.controller.kp_tilt * normalized_err_y * 90.0  # Inverted to match actual control
                
                # Determine expected direction for debugging
                expected_pan_dir = "right" if error_x > 0 else "left" if error_x < 0 else "center"
                expected_tilt_dir = "down" if error_y > 0 else "up" if error_y < 0 else "center"
                actual_pan_dir = "right" if pan_angle > 90 else "left" if pan_angle < 90 else "center"
                actual_tilt_dir = "down" if tilt_angle > 90 else "up" if tilt_angle < 90 else "center"
                
                # Log less frequently to reduce spam
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 3 == 0:  # Log every 3rd update for better debugging
                    self.get_logger().info(
                        f'Ball: ({ball_center_x:.1f}, {ball_center_y:.1f}), '
                        f'Laser: ({laser_x:.1f}, {laser_y:.1f}), '
                        f'err: ({error_x:.1f}, {error_y:.1f}), '
                        f'exp: pan→{expected_pan_dir} (Δ{expected_delta_pan:.2f}°), tilt→{expected_tilt_dir} (Δ{expected_delta_tilt:.2f}°), '
                        f'act: pan={pan_angle:.1f}° ({actual_pan_dir}), tilt={tilt_angle:.1f}° ({actual_tilt_dir})'
                    )
                
                self.last_detection_time = current_time
            else:
                # Coordinates are stale, reset to center
                if current_time - self.last_detection_time > self.timeout:
                    self.controller.reset_to_center()
                    if self.servo is not None:
                        self.servo.set_pan(90.0)
                        self.servo.set_tilt(90.0)
                    self.get_logger().info(f"No detection for {self.timeout}s, resetting to center")
                    self.last_detection_time = current_time
                
                # Publish "none" for laser position when no ball detected
                laser_msg = String()
                laser_msg.data = "none"
                self.laser_pub.publish(laser_msg)
        else:
            # No coordinates available, check if we should reset
            if current_time - self.last_detection_time > self.timeout:
                self.controller.reset_to_center()
                if self.servo is not None:
                    self.servo.set_pan(90.0)
                    self.servo.set_tilt(90.0)
                # Only log once per timeout period
                if int(current_time) % int(self.timeout + 1) == 0:
                    self.get_logger().info(f"No detection for {self.timeout}s, resetting to center")
                self.last_detection_time = current_time
            
            # Publish "none" for laser position when no ball detected
            laser_msg = String()
            laser_msg.data = "none"
            self.laser_pub.publish(laser_msg)


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
