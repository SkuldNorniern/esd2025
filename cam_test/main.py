#!/usr/bin/env python3
import os, sys, time, signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Publish compressed stream (standard topic name)
        self.pub = self.create_publisher(CompressedImage, "/image/compressed", 10)

        self.device_path = "/dev/video1"
        self.req_w = 640
        self.req_h = 480
        self.req_fps = 30

        self.get_logger().info(f"Opening {self.device_path} (MJPG)...")
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open {self.device_path}")

        # Ask for MJPG + exact supported size
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.req_fps)

        # Reduce latency / stale frames
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Read back what we actually got
        self.width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps    = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc_code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

        self.get_logger().info(f"Camera negotiated: {self.width}x{self.height} {fourcc_str} fps={self.fps}")

        if fourcc_str != "MJPG":
            self.get_logger().warn(f"Camera did not negotiate MJPG (got {fourcc_str}). Expect bandwidth issues.")

        # Warm up / flush junk frames
        for _ in range(30):
            self.cap.grab()

        self.frame_count = 0
        self.bad_count = 0

        self.timer = self.create_timer(1.0 / 30.0, self.tick)

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

    def _sig(self, *_):
        self.get_logger().info("Shutting down...")
        try:
            self.cap.release()
        finally:
            rclpy.shutdown()
            sys.exit(0)

    def tick(self):
        # Grab+retrieve can behave better than read() on some V4L2 backends
        if not self.cap.grab():
            self.bad_count += 1
            return

        ok, frame = self.cap.retrieve()
        if not ok or frame is None:
            self.bad_count += 1
            return

        # Validate dimensions (skip corrupted decodes)
        h, w = frame.shape[:2]
        if (w, h) != (self.width, self.height) or frame.size == 0:
            self.bad_count += 1
            return

        # Encode to JPEG for ROS transport
        enc_ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not enc_ok or jpeg is None or len(jpeg) < 100:
            self.bad_count += 1
            return

        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()

        self.pub.publish(msg)
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Published #{self.frame_count} JPEG ({len(msg.data)} bytes), bad={self.bad_count}"
            )

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
