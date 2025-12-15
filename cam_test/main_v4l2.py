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
    try:
        from v4l2py.device import BufferType, PixelFormat
    except ImportError:
        # v4l2py 3.0+ uses linuxpy
        try:
            from linuxpy.video.device import BufferType, PixelFormat
        except ImportError:
            # Fallback - will work without these
            BufferType = None
            PixelFormat = None
            print("Warning: Could not import BufferType/PixelFormat (v4l2py API changed)")
except ImportError:
    print("ERROR: v4l2py not installed!")
    print("Install with: pip install v4l2py")
    print("Or use: uv pip install v4l2py")
    sys.exit(1)


class CameraNodeV4L2(Node):
    def __init__(self):
        super().__init__("camera_node_v4l2")
        
        # Configuration
        self.device_path = os.environ.get("CAM_DEVICE_PATH", "0")
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
        # v4l2py accepts: int (0, 1, 2...) or string ("/dev/video0", "/dev/video1"...)
        self.get_logger().info(f"Opening camera {self.device_path} with v4l2py...")
        try:
            # Try to convert to int if it's a numeric string
            try:
                device_id = int(self.device_path)
            except ValueError:
                device_id = self.device_path
            
            self.cam = Device.from_id(device_id)
            self.cam.open()
        except Exception as e:
            self.get_logger().error(f"Failed to open camera: {e}")
            self.get_logger().error(f"Try a different device:")
            self.get_logger().error(f"  export CAM_DEVICE_PATH=0   # or 1, 2, etc")
            self.get_logger().error(f"  export CAM_DEVICE_PATH=/dev/video0")
            raise RuntimeError(f"Cannot open {self.device_path}")
        
        # Get camera info
        self.get_logger().info(f"Camera: {self.cam.info.card}")
        
        # Find MJPEG format
        # Note: v4l2py 3.0 returns format info differently
        mjpeg_supported = False
        available_formats = []
        
        try:
            for fmt in self.cam.info.formats:
                # Try different attribute names (API changed in v3.0)
                fmt_name = None
                if hasattr(fmt, 'pixelformat'):
                    fmt_name = fmt.pixelformat
                elif hasattr(fmt, 'pixel_format'):
                    fmt_name = fmt.pixel_format
                elif hasattr(fmt, 'description'):
                    fmt_name = fmt.description
                
                available_formats.append(str(fmt_name))
                
                # Check if MJPEG/JPEG
                if fmt_name:
                    fmt_str = str(fmt_name).upper()
                    if 'MJPEG' in fmt_str or 'MJPG' in fmt_str or 'JPEG' in fmt_str:
                        mjpeg_supported = True
                        break
        except Exception as e:
            self.get_logger().warn(f"Error checking formats: {e}")
            # Try to continue anyway
            mjpeg_supported = True
        
        if not mjpeg_supported and available_formats:
            self.get_logger().error("Camera does not support MJPEG!")
            self.get_logger().error(f"Available formats: {available_formats}")
            self.get_logger().error("Try a different device or check camera capabilities")
            raise RuntimeError("MJPEG not supported")
        
        self.get_logger().info("✓ Camera supports MJPEG (or attempting to use it)")
        
        # Set format - try multiple methods for v4l2py compatibility
        format_set = False
        if BufferType and PixelFormat:
            try:
                # Method 1: Try with PixelFormat enum
                self.cam.set_format(BufferType.VIDEO_CAPTURE, self.req_w, self.req_h, PixelFormat.MJPEG)
                format_set = True
            except Exception as e1:
                try:
                    # Method 2: Try with string
                    self.cam.set_format(BufferType.VIDEO_CAPTURE, self.req_w, self.req_h, "MJPEG")
                    format_set = True
                except Exception as e2:
                    self.get_logger().warn(f"Could not set format explicitly: {e1}")
        
        if not format_set:
            self.get_logger().warn("Using camera's default format (v4l2py API compatibility mode)")
        
        # Get actual format
        try:
            if BufferType:
                fmt = self.cam.get_format(BufferType.VIDEO_CAPTURE)
            else:
                # Fallback: try numeric value for VIDEO_CAPTURE
                fmt = self.cam.get_format(1)  # BufferType.VIDEO_CAPTURE = 1
            self.width = fmt.width
            self.height = fmt.height
            
            # Try to get pixel format name
            if hasattr(fmt, 'pixelformat'):
                fmt_name = fmt.pixelformat
            elif hasattr(fmt, 'pixel_format'):
                fmt_name = fmt.pixel_format
            else:
                fmt_name = "UNKNOWN"
            
            self.get_logger().info(f"Camera negotiated: {self.width}x{self.height} {fmt_name}")
            
            if self.width != self.req_w or self.height != self.req_h:
                self.get_logger().warn(
                    f"Camera negotiated {self.width}x{self.height} instead of {self.req_w}x{self.req_h}. "
                    "This is normal - camera may not support exact resolution."
                )
        except Exception as e:
            self.get_logger().warn(f"Could not query format: {e}")
            # Use requested dimensions as fallback
            self.width = self.req_w
            self.height = self.req_h
            self.get_logger().info(f"Using requested dimensions: {self.width}x{self.height}")
        
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

