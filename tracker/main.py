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
        # Proportional gains - increased for better tracking
        self.kp_pan = 0.15
        self.kp_tilt = 0.15
        
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
        self.max_rate = 10.0  # Max 10 degrees per update for faster response
        
        self.last_update = time.time()
    
    def update(self, x1: float, y1: float, x2: float, y2: float) -> Tuple[float, float]:
        """Update servo positions based on ball detection"""
        # Calculate ball center
        ball_center_x = (x1 + x2) / 2.0
        ball_center_y = (y1 + y2) / 2.0
        
        # Calculate image center
        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0
        
        # Calculate error (pixels from center)
        error_x = ball_center_x - image_center_x
        error_y = ball_center_y - image_center_y
        
        # Normalize error to -1.0 to 1.0 range
        normalized_error_x = error_x / image_center_x
        normalized_error_y = error_y / image_center_y
        
        # Apply P-control
        # Positive error_x means ball is to the right, need to pan right (increase angle)
        # Positive error_y means ball is below center, need to tilt down (increase angle)
        # Note: If servos are mounted backwards, invert the signs
        delta_pan = self.kp_pan * normalized_error_x * 90.0  # Scale to degrees
        delta_tilt = self.kp_tilt * normalized_error_y * 90.0
        
        # Invert pan direction if servos are mounted backwards
        # Based on logs, pan is going wrong direction, so invert it
        delta_pan = -delta_pan
        
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
                
                # Log less frequently to reduce spam
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 5 == 0:  # Log every 5th update
                    self.get_logger().info(
                        f'Ball center: ({ball_center_x:.1f}, {ball_center_y:.1f}), '
                        f'error: ({error_x:.1f}, {error_y:.1f}) -> '
                        f'Pan: {pan_angle:.1f}°, Tilt: {tilt_angle:.1f}°'
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
