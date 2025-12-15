#!/usr/bin/env python3
"""
Camera test node - Python version
Captures frames from /dev/video2 using V4L2 and publishes to ROS2 /image topic

Requirements:
- ROS2 installed and sourced (source /opt/ros/<distro>/setup.bash)
- rclpy installed: sudo apt install ros-<distro>-rclpy
- opencv-python: pip install opencv-python
"""

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    RCLPY_AVAILABLE = True
except ImportError as e:
    RCLPY_AVAILABLE = False
    print(f"Error: rclpy not available: {e}")
    print("To install ROS2 Python packages:")
    print("  1. Source ROS2: source /opt/ros/<distro>/setup.bash")
    print("  2. Install rclpy: sudo apt install ros-<distro>-rclpy")
    print("  3. Common distros: humble, foxy, galactic, iron")
    print()
    print("Alternatively, use the Rust version: cargo run --release -p cam_test")
    sys.exit(1)

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Error: opencv-python not available")
    print("  Install with: pip install opencv-python")
    sys.exit(1)

import time
import sys
import signal
import os


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Create publisher for /image topic
        self.publisher = self.create_publisher(Image, '/image', 10)
        
        # Camera settings
        self.device_path = '/dev/video2'
        self.width = 640
        self.height = 640
        self.fps = 30
        
        # Initialize camera
        self.get_logger().info('Initializing camera node with V4L2...')
        self.get_logger().info(f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "0")}')
        
        # Wait for publisher to establish connections
        self.get_logger().info('Waiting for publisher to establish connections with subscribers...')
        self.get_logger().info('  (DDS discovery can take 5-10 seconds for cross-device communication)')
        for i in range(10):
            time.sleep(1)
            print('.', end='', flush=True)
        print()
        
        # Open camera
        self.get_logger().info('Opening camera device...')
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device: {self.device_path}')
            sys.exit(1)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Try to set MJPEG format (FourCC: MJPG)
        # Note: Not all cameras support this via OpenCV, but we'll try
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Get actual format
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        fourcc_code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])
        
        self.get_logger().info(f'Found camera device: {self.device_path}')
        self.get_logger().info(f'Camera format: {actual_width}x{actual_height} {fourcc_str}')
        
        if fourcc_str == 'MJPG':
            self.format_is_mjpeg = True
            self.get_logger().info('Format: MJPEG (compressed JPEG)')
        else:
            self.format_is_mjpeg = False
            self.get_logger().info(f'Format: {fourcc_str} (may need conversion)')
        
        # Wait for camera to stabilize
        self.get_logger().info('Camera started, waiting for stream to stabilize...')
        for i in range(20):
            time.sleep(0.1)
            if i % 5 == 0:
                print('.', end='', flush=True)
        print()
        
        if self.format_is_mjpeg:
            self.get_logger().info('Publishing MJPEG (compressed JPEG) images to /image topic (press Ctrl+C to stop)')
        else:
            self.get_logger().info('Publishing images to /image topic (press Ctrl+C to stop)')
        
        # Create timer for frame capture (30 FPS = ~33ms)
        self.frame_count = 0
        self.timer = self.create_timer(1.0 / self.fps, self.capture_and_publish)
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info('Shutting down...')
        if self.cap.isOpened():
            self.cap.release()
        rclpy.shutdown()
        sys.exit(0)
    
    def capture_and_publish(self):
        """Capture frame and publish to ROS topic"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        self.frame_count += 1
        
        # Log periodically
        if self.frame_count <= 5 or self.frame_count % 30 == 0:
            self.get_logger().info(f'Captured frame #{self.frame_count}')
        
        # For MJPEG, we need to encode the frame to JPEG
        # For other formats, we'll send as RGB8
        if self.format_is_mjpeg:
            # Encode frame to JPEG
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
            result, jpeg_data = cv2.imencode('.jpg', frame, encode_params)
            
            if not result:
                self.get_logger().error('Failed to encode frame to JPEG')
                return
            
            # Create ROS Image message with JPEG data
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            msg.height = self.height
            msg.width = self.width
            msg.encoding = 'jpeg'
            msg.is_bigendian = 0
            msg.step = 0  # Compressed format has no step
            msg.data = jpeg_data.tobytes()
        else:
            # Convert BGR to RGB (OpenCV uses BGR by default)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Create ROS Image message with RGB8 data
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            msg.height = rgb_frame.shape[0]
            msg.width = rgb_frame.shape[1]
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = rgb_frame.shape[1] * 3  # 3 bytes per pixel (RGB)
            msg.data = rgb_frame.tobytes()
        
        # Publish message
        self.publisher.publish(msg)
        
        # Log periodically
        if self.frame_count % 30 == 0:
            if self.format_is_mjpeg:
                self.get_logger().info(f'Published frame #{self.frame_count}: {msg.width}x{msg.height} MJPEG ({len(msg.data)} bytes compressed)')
            else:
                self.get_logger().info(f'Published frame #{self.frame_count}: {msg.width}x{msg.height} RGB8 ({len(msg.data)} bytes)')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and node.cap.isOpened():
            node.cap.release()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
