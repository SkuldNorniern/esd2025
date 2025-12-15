#!/usr/bin/env python3
# Optimized for Raspberry Pi 3 - reduced resolution, FPS, and frame skipping
# Configuration matches config.toml defaults used by Rust version
# Environment variables for tuning:
#   CAM_DEVICE_PATH: Camera device path (default: /dev/video0)
#   CAM_WIDTH, CAM_HEIGHT: Resolution (default: 512x512 to match config.toml)
#   CAM_FPS: Target FPS (default: 15)
#   JPEG_QUALITY: JPEG compression quality 1-100 (default: 65)
#   PUBLISH_EVERY_N: Publish every Nth frame (default: 3 to match config.toml)
#   PUBLISH_RAW: Set to "1" to enable raw RGB8 publishing (default: "0")
#   CAM_WARMUP: Number of frames to discard on startup (default: 10)
import os, sys, time, signal, warnings

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage

import cv2
import numpy as np

# Optional: suppress libjpeg "Corrupt JPEG data" spam on stderr
class StderrFilter:
    def __init__(self, original_stderr):
        self.original_stderr = original_stderr
    def write(self, message):
        if ("Corrupt JPEG data" in message) or ("premature end of data segment" in message):
            return
        self.original_stderr.write(message)
    def flush(self):
        self.original_stderr.flush()
    def __getattr__(self, name):
        return getattr(self.original_stderr, name)

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Keep original /image publisher (raw)
        self.pub_raw = self.create_publisher(Image, "/image", 10)
        # Add proper compressed stream (JPEG)
        self.pub_jpeg = self.create_publisher(CompressedImage, "/image/compressed", 10)

        # Match config.toml defaults - device_path can be overridden with CAM_DEVICE_PATH
        self.device_path = os.environ.get("CAM_DEVICE_PATH", "/dev/video0")
        # Reduced resolution for Pi 3 - match config.toml defaults (512x512)
        self.req_w = int(os.environ.get("CAM_WIDTH", "512"))
        self.req_h = int(os.environ.get("CAM_HEIGHT", "512"))
        self.req_fps = int(os.environ.get("CAM_FPS", "15"))  # Reduced FPS for Pi 3
        # Lower JPEG quality for smaller file sizes - can be overridden with JPEG_QUALITY env var
        self.jpeg_quality = int(os.environ.get("JPEG_QUALITY", "65"))

        # Disable raw by default for Pi 3 - set PUBLISH_RAW=1 to enable
        self.publish_raw = os.environ.get("PUBLISH_RAW", "0") != "0"

        # Filter stderr spam (optional)
        self._orig_stderr = sys.stderr
        sys.stderr = StderrFilter(self._orig_stderr)

        self.get_logger().info(f"Opening camera {self.device_path} with V4L2...")
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera device: {self.device_path}")

        # Ask for MJPG
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.req_fps)

        # Reduce internal buffering / latency (may or may not be supported)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Read actual negotiated settings
        self.width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps    = float(self.cap.get(cv2.CAP_PROP_FPS))
        fourcc_code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

        self.get_logger().info(f"Camera negotiated: {self.width}x{self.height} {fourcc_str} fps={self.fps:.1f}")
        if fourcc_str != "MJPG":
            self.get_logger().warn(f"Camera did not negotiate MJPG (got {fourcc_str}). You may get bandwidth issues.")
        
        # Warn if resolution doesn't match requested
        if self.width != self.req_w or self.height != self.req_h:
            self.get_logger().warn(
                f"Camera negotiated {self.width}x{self.height} instead of requested {self.req_w}x{self.req_h}. "
                f"This is normal - camera may not support exact resolution."
            )

        # Warm up / flush junk frames (reduced for faster startup)
        warmup_frames = int(os.environ.get("CAM_WARMUP", "10"))
        for _ in range(warmup_frames):
            self.cap.grab()

        self.frame_count = 0
        self.bad_count = 0
        # Publish every Nth frame - higher value = less network traffic
        # Can be overridden with PUBLISH_EVERY_N env var (default: 3 for Pi 3)
        self.publish_every_n = int(os.environ.get("PUBLISH_EVERY_N", "3"))

        period = 1.0 / max(1.0, self.req_fps)
        self.timer = self.create_timer(period, self.capture_and_publish)

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.get_logger().info("")
        self.get_logger().info("ROS Configuration:")
        self.get_logger().info(f"  ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
        self.get_logger().info("")
        self.get_logger().info("Camera Configuration:")
        self.get_logger().info(f"  Device: {self.device_path}")
        self.get_logger().info(f"  Resolution: {self.width}x{self.height}")
        self.get_logger().info(f"  FPS: {self.fps:.1f}")
        self.get_logger().info(f"  Format: {fourcc_str}")
        self.get_logger().info(f"  Publish every Nth frame: {self.publish_every_n}")
        self.get_logger().info(f"  JPEG quality: {self.jpeg_quality}")
        self.get_logger().info(f"  Publish raw RGB8: {self.publish_raw}")
        self.get_logger().info("")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /image            (sensor_msgs/Image, rgb8)  [if PUBLISH_RAW=1]")
        self.get_logger().info("  /image/compressed (sensor_msgs/CompressedImage, jpeg)")
        self.get_logger().info("")
        self.get_logger().info("Press Ctrl+C to stop.")

    def _sig(self, *_):
        self.get_logger().info("Shutting down...")
        try:
            if self.cap.isOpened():
                self.cap.release()
        finally:
            sys.stderr = self._orig_stderr
            rclpy.shutdown()
            sys.exit(0)

    def capture_and_publish(self):
        # grab/retrieve is often more stable than read() on V4L2
        if not self.cap.grab():
            self.bad_count += 1
            return
        ok, frame = self.cap.retrieve()
        if not ok or frame is None or frame.size == 0:
            self.bad_count += 1
            return

        h, w = frame.shape[:2]
        if (w != self.width) or (h != self.height):
            # Drop weird frames (dimension mismatch usually means corruption)
            self.bad_count += 1
            return

        stamp = self.get_clock().now().to_msg()

        # Publish throttling (reduce network overhead)
        self.frame_count += 1
        if self.frame_count % self.publish_every_n != 0:
            return

        # 1) Publish raw /image (RGB8) for compatibility with your existing pipeline
        if self.publish_raw:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = Image()
            msg.header = Header()
            msg.header.stamp = stamp
            msg.header.frame_id = "camera_frame"
            msg.height = h
            msg.width = w
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = w * 3
            msg.data = rgb.tobytes()
            self.pub_raw.publish(msg)

        # 2) Publish compressed JPEG on /image/compressed
        # Note: If camera provides MJPEG directly, we could use it without re-encoding
        # But OpenCV doesn't expose raw MJPEG easily, so we re-encode
        # For better performance, use the Rust version which uses raw MJPEG from camera
        enc_ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not enc_ok or jpeg is None or len(jpeg) < 100:
            self.bad_count += 1
            return

        cmsg = CompressedImage()
        cmsg.header = Header()
        cmsg.header.stamp = stamp
        cmsg.header.frame_id = "camera_frame"
        cmsg.format = "jpeg"
        cmsg.data = jpeg.tobytes()
        self.pub_jpeg.publish(cmsg)
        
        # Log first few frames and then periodically
        if self.frame_count <= 5 or self.frame_count % 60 == 0:
            self.get_logger().info(
                f"Published frame #{self.frame_count}: {w}x{h} MJPEG ({len(cmsg.data)} bytes compressed, "
                f"skip={self.publish_every_n}, bad={self.bad_count})"
            )

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
