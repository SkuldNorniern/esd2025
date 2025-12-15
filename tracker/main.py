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
        self.kp_pan = 0.20
        self.kp_tilt = 0.20
        
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
        self.max_rate = 8.0  # Max 8 degrees per update for faster response
        
        self.last_update = time.time()
        
        # Track previous ball position to detect if ball is actually moving
        self.last_ball_center_x: Optional[float] = None
        self.last_ball_center_y: Optional[float] = None
        self.position_change_threshold = 3.0  # Pixels - only move if ball moved more than this (reduced for accuracy)
    
    def update(self, x1: float, y1: float, x2: float, y2: float) -> Tuple[float, float]:
        """Update servo positions based on ball detection"""
        # Calculate ball center
        ball_center_x = (x1 + x2) / 2.0
        ball_center_y = (y1 + y2) / 2.0
        
        # Calculate image center and error early
        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0
        error_x = ball_center_x - image_center_x
        error_y = ball_center_y - image_center_y
        error_magnitude = (error_x**2 + error_y**2)**0.5
        
        # Check if ball position actually changed significantly
        # But allow movement if error is large (ball is far from center)
        if self.last_ball_center_x is not None and self.last_ball_center_y is not None:
            ball_moved_x = abs(ball_center_x - self.last_ball_center_x)
            ball_moved_y = abs(ball_center_y - self.last_ball_center_y)
            
            # If ball hasn't moved much AND error is small, don't update servos
            # But if error is large (ball is far from center), always update
            # Also, always allow movement if ball is on the left (error_x < 0) to fix left-side issue
            # And always allow movement if ball is above center (error_y < 0) to fix tilt issue
            if (ball_moved_x < self.position_change_threshold and 
                ball_moved_y < self.position_change_threshold and
                error_magnitude < 50.0 and
                error_x >= 0 and
                error_y >= 0):  # Don't skip updates when ball is on left or above center
                # Ball is stationary and near center on right/bottom side, keep current servo positions
                return (self.pan_angle, self.tilt_angle)
        
        # Update last known ball position
        self.last_ball_center_x = ball_center_x
        self.last_ball_center_y = ball_center_y
        
        # Normalize error to -1.0 to 1.0 range
        normalized_error_x = error_x / image_center_x
        normalized_error_y = error_y / image_center_y
        
        # Calculate direct angle correction based on error
        # Map image error to servo angle change
        # For accurate pointing, we want direct proportional control
        
        # Pan control:
        # error_x > 0: ball is right of center -> pan right (increase angle toward 180°)
        # error_x < 0: ball is left of center -> pan left (decrease angle toward 0°)
        # Apply different gains for left vs right to compensate for asymmetry
        if normalized_error_x > 0:
            # Ball is on the right - reduce movement by 5%
            delta_pan = self.kp_pan * normalized_error_x * 90.0 * 0.95  # 5% less movement
        elif normalized_error_x < 0:
            # Ball is on the left - ensure it moves (no dead zone for left side)
            delta_pan = self.kp_pan * normalized_error_x * 90.0
        else:
            delta_pan = 0.0
        
        # Tilt control:
        # error_y > 0: ball is below center -> tilt down (increase angle toward 180°)
        # error_y < 0: ball is above center -> tilt up (decrease angle toward 0°)
        # Image coordinates: (0,0) is top-left, y increases downward
        # gpiozero Servo mapping: -1.0 = 0° (up), 0.0 = 90° (center), 1.0 = 180° (down)
        # If tilt keeps going up (angle always decreasing), the direction might be inverted
        # When error_y < 0 (ball above): we want tilt up (decrease angle) -> delta_tilt should be negative
        # Direct: delta_tilt = kp * (-error_y) = negative -> angle decreases -> tilt up (correct!)
        # But if it always goes up regardless, maybe we need to check if error_y is always negative
        # Or maybe the control needs inversion. Try inverting to see if it fixes the issue
        delta_tilt = -self.kp_tilt * normalized_error_y * 90.0  # Inverted to fix "always going up" issue
        
        # Add small dead zone only for very small errors to prevent micro-movements
        # But don't apply dead zone to left side to ensure it moves
        dead_zone = 0.05  # 5% of image size (about 32 pixels for 640x640)
        if abs(normalized_error_x) < dead_zone and normalized_error_x >= 0:
            # Only apply dead zone for right side or center
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
    
    def control_loop(self) -> None:
        """Main control loop - runs at 10Hz"""
        current_time = time.time()
        
        # Check for ball detection coordinates
        if self.coords is not None:
            # Check if coordinates are stale
            if current_time - self.last_coords_update < self.timeout:
                x1, y1, x2, y2 = self.coords
                pan_angle, tilt_angle = self.controller.update(x1, y1, x2, y2)
                
                if self.servo is not None:
                    self.servo.set_pan(pan_angle)
                    self.servo.set_tilt(tilt_angle)
                
                # Calculate ball center and error for better logging
                ball_center_x = (x1 + x2) / 2.0
                ball_center_y = (y1 + y2) / 2.0
                error_x = ball_center_x - (self.image_width / 2.0)
                error_y = ball_center_y - (self.image_height / 2.0)
                
                # Calculate what the delta should be for debugging
                normalized_err_x = error_x / (self.image_width / 2.0)
                normalized_err_y = error_y / (self.image_height / 2.0)
                expected_delta_pan = self.controller.kp_pan * normalized_err_x * 90.0
                if normalized_err_x > 0:
                    expected_delta_pan *= 0.95  # Right side is 5% less
                expected_delta_tilt = self.controller.kp_tilt * normalized_err_y * 90.0
                
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
