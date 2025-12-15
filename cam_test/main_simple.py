#!/usr/bin/env python3
"""
Simplest Possible Camera Node - Just OpenCV + ROS2
No GUI, no complexity, just works!
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__("simple_camera")
        
        # Configuration
        device = int(os.environ.get("CAM_DEVICE", "0"))
        self.width = int(os.environ.get("CAM_WIDTH", "640"))
        self.height = int(os.environ.get("CAM_HEIGHT", "480"))
        self.fps = int(os.environ.get("CAM_FPS", "15"))
        self.quality = int(os.environ.get("JPEG_QUALITY", "65"))
        self.skip = int(os.environ.get("PUBLISH_EVERY_N", "3"))
        
        # Open camera
        self.get_logger().info(f"Opening camera {device}...")
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        
        # Set MJPEG format
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Check what we got
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera: {actual_w}x{actual_h} @ {self.fps} FPS")
        
        # Publisher
        self.pub = self.create_publisher(Image, "/image", 10)
        
        # Timer
        self.frame_count = 0
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.capture)
        
        self.get_logger().info("Camera ready!")
        self.get_logger().info(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
    
    def capture(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        self.frame_count += 1
        
        # Skip frames
        if self.frame_count % self.skip != 0:
            return
        
        # Encode as JPEG
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        
        # Create ROS message
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "jpeg"
        msg.is_bigendian = 0
        msg.step = 0
        msg.data = jpeg.tobytes()
        
        # Publish
        self.pub.publish(msg)
        
        if self.frame_count % 60 == 0:
            self.get_logger().info(f"Published frame #{self.frame_count}: {len(jpeg)} bytes")

def main():
    rclpy.init()
    node = SimpleCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

