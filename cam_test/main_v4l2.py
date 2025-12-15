#!/usr/bin/env python3
"""
Camera Node using v4l2py - More efficient than OpenCV version
Direct V4L2 access, no re-encoding, better MJPEG handling
Perfect for Raspberry Pi - headless friendly

Install: pip install v4l2py

Environment variables:
  CAM_DEVICE_PATH: Camera device path (default: /dev/video1)
  CAM_WIDTH, CAM_HEIGHT: Resolution (default: 512x512)
  PUBLISH_EVERY_N: Publish every Nth frame (default: 3)
  JPEG_QUALITY: Not used (v4l2py uses camera's native MJPEG)
"""

import os
import sys
import time
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image

try:
    from v4l2py import Device
    from v4l2py.device import BufferType, PixelFormat
except ImportError:
    print("ERROR: v4l2py not installed!")
    print("Install with: pip install v4l2py")
    print("Or use: uv pip install v4l2py")
    sys.exit(1)


class CameraNodeV4L2(Node):
    def __init__(self):
        super().__init__("camera_node_v4l2")
        
        # Configuration
        self.device_path = os.environ.get("CAM_DEVICE_PATH", "/dev/video1")
        self.req_w = int(os.environ.get("CAM_WIDTH", "512"))
        self.req_h = int(os.environ.get("CAM_HEIGHT", "512"))
        self.publish_every_n = int(os.environ.get("PUBLISH_EVERY_N", "3"))
        
        # State
        self.frame_count = 0
        self.bad_count = 0
        self.running = True
        
        # Publisher
        self.pub_image = self.create_publisher(Image, "/image", 10)
        
        # Open camera
        self.get_logger().info(f"Opening camera {self.device_path} with v4l2py...")
        try:
            self.cam = Device.from_id(self.device_path)
            self.cam.open()
        except Exception as e:
            self.get_logger().error(f"Failed to open camera: {e}")
            raise RuntimeError(f"Cannot open {self.device_path}")
        
        # Get camera info
        self.get_logger().info(f"Camera: {self.cam.info.card}")
        
        # Find MJPEG format
        mjpeg_format = None
        for fmt in self.cam.info.formats:
            if fmt.pixelformat == PixelFormat.MJPEG:
                mjpeg_format = fmt
                break
        
        if not mjpeg_format:
            self.get_logger().error("Camera does not support MJPEG!")
            self.get_logger().error(f"Available formats: {[f.pixelformat for f in self.cam.info.formats]}")
            raise RuntimeError("MJPEG not supported")
        
        self.get_logger().info("✓ Camera supports MJPEG")
        
        # Set format
        try:
            self.cam.set_format(BufferType.VIDEO_CAPTURE, self.req_w, self.req_h, PixelFormat.MJPEG)
        except Exception as e:
            self.get_logger().warn(f"Failed to set exact format: {e}")
        
        # Get actual format
        fmt = self.cam.get_format(BufferType.VIDEO_CAPTURE)
        self.width = fmt.width
        self.height = fmt.height
        self.pixelformat = fmt.pixelformat
        
        self.get_logger().info(f"Camera negotiated: {self.width}x{self.height} {self.pixelformat.name}")
        
        if self.width != self.req_w or self.height != self.req_h:
            self.get_logger().warn(
                f"Camera negotiated {self.width}x{self.height} instead of {self.req_w}x{self.req_h}. "
                "This is normal - camera may not support exact resolution."
            )
        
        if self.pixelformat != PixelFormat.MJPEG:
            self.get_logger().error(f"Camera did not negotiate MJPEG (got {self.pixelformat.name})!")
            raise RuntimeError("MJPEG required but not negotiated")
        
        # Log configuration
        self.get_logger().info("")
        self.get_logger().info("ROS Configuration:")
        self.get_logger().info(f"  ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
        self.get_logger().info("")
        self.get_logger().info("Camera Configuration:")
        self.get_logger().info(f"  Device: {self.device_path}")
        self.get_logger().info(f"  Resolution: {self.width}x{self.height}")
        self.get_logger().info(f"  Format: MJPEG (native from camera, no re-encoding!)")
        self.get_logger().info(f"  Publish every Nth frame: {self.publish_every_n}")
        self.get_logger().info("")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /image (sensor_msgs/Image, encoding='jpeg', MJPEG native)")
        self.get_logger().info("")
        self.get_logger().info("Press Ctrl+C to stop.")
        self.get_logger().info("")
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)
        
        # Start capture timer (run at ~30Hz, but only publish every Nth frame)
        period = 1.0 / 30.0
        self.timer = self.create_timer(period, self.capture_and_publish)
    
    def _sig(self, *_):
        self.get_logger().info("Shutting down...")
        self.running = False
        try:
            self.cam.close()
        except:
            pass
        rclpy.shutdown()
        sys.exit(0)
    
    def capture_and_publish(self):
        if not self.running:
            return
        
        try:
            # Capture frame using v4l2py
            # This gets raw MJPEG data directly from camera - no decoding/re-encoding!
            frame = next(iter(self.cam))
            
            # frame.data is already MJPEG bytes - perfect!
            frame_data = bytes(frame.data)
            
            # Check if valid JPEG (starts with FF D8)
            if len(frame_data) < 2 or frame_data[0] != 0xFF or frame_data[1] != 0xD8:
                self.bad_count += 1
                return
            
            self.frame_count += 1
            
            # Throttle publishing
            if self.frame_count % self.publish_every_n != 0:
                return
            
            # Create ROS message
            stamp = self.get_clock().now().to_msg()
            
            msg = Image()
            msg.header = Header()
            msg.header.stamp = stamp
            msg.header.frame_id = "camera_frame"
            msg.height = self.height
            msg.width = self.width
            msg.encoding = "jpeg"  # MJPEG from camera
            msg.is_bigendian = 0
            msg.step = 0  # Compressed format
            msg.data = frame_data
            
            self.pub_image.publish(msg)
            
            # Log periodically
            if self.frame_count <= 5 or self.frame_count % 60 == 0:
                self.get_logger().info(
                    f"Published frame #{self.frame_count}: {self.width}x{self.height} "
                    f"MJPEG ({len(frame_data)} bytes native, skip={self.publish_every_n}, bad={self.bad_count})"
                )
        
        except StopIteration:
            # No more frames (camera stopped)
            self.get_logger().error("Camera stopped providing frames")
            self.running = False
        except Exception as e:
            self.bad_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().warn(f"Frame capture error: {e}")


def main():
    rclpy.init()
    
    try:
        node = CameraNodeV4L2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)
    finally:
        try:
            node.cam.close()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

