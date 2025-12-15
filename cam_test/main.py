#!/usr/bin/env python3
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

        self.device_path = "/dev/video1"
        self.req_w = 640
        self.req_h = 480          # Don’t request 640x640 unless the camera actually supports it
        self.req_fps = 30
        self.jpeg_quality = 85

        # If you really want to cut CPU, set PUBLISH_RAW=0 and only publish /image/compressed
        self.publish_raw = os.environ.get("PUBLISH_RAW", "1") != "0"

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

        # Warm up / flush junk frames
        for _ in range(30):
            self.cap.grab()

        self.frame_count = 0
        self.bad_count = 0

        period = 1.0 / max(1.0, self.req_fps)
        self.timer = self.create_timer(period, self.capture_and_publish)

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.get_logger().info("Publishing:")
        self.get_logger().info("  /image            (sensor_msgs/Image, rgb8)  [if PUBLISH_RAW=1]")
        self.get_logger().info("  /image/compressed (sensor_msgs/CompressedImage, jpeg)")
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

        # 2) Publish compressed JPEG on /image/compressed (this is the “MJPEG over ROS” path)
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

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Published #{self.frame_count} {w}x{h} jpeg={len(cmsg.data)}B bad={self.bad_count}"
            )

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
