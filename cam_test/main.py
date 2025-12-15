#!/usr/bin/env python3
# Optimized for Raspberry Pi 3 - reduced resolution, FPS, and frame skipping
# Configuration matches config.toml defaults used by Rust version
# Publishes to /image topic with sensor_msgs/Image (encoding="jpeg")
# Environment variables for tuning:
#   CAM_DEVICE_PATH: Camera device path (default: /dev/video0)
#   CAM_WIDTH, CAM_HEIGHT: Resolution (default: 512x512 to match config.toml)
#   CAM_FPS: Target FPS (default: 15)
#   JPEG_QUALITY: JPEG compression quality 1-100 (default: 65)
#   PUBLISH_EVERY_N: Publish every Nth frame (default: 3 to match config.toml)
#   CAM_WARMUP: Number of frames to discard on startup (default: 10)
#   SHOW_TRACKING_VIEW: Set to "1" to enable real-time tracking visualization (default: "0")
import os, sys, time, signal, warnings
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image

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

def parse_ball_coords(s: str) -> Optional[Tuple[float, float, float, float]]:
    """Parse ball coordinates string: 'x1,y1,x2,y2' or 'none'"""
    if s == "none":
        return None
    parts = s.split(',')
    if len(parts) != 4:
        return None
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
    except ValueError:
        return None


def parse_laser_pos(s: str) -> Optional[Tuple[float, float]]:
    """Parse laser position string: 'x,y' or 'none'"""
    if s == "none":
        return None
    parts = s.split(',')
    if len(parts) != 2:
        return None
    try:
        return (float(parts[0]), float(parts[1]))
    except ValueError:
        return None


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Publisher for /image topic - matches Rust version behavior
        # Publishes sensor_msgs/Image with encoding="jpeg" (MJPEG from camera)
        self.pub_image = self.create_publisher(Image, "/image", 10)

        # Match config.toml defaults - device_path can be overridden with CAM_DEVICE_PATH
        self.device_path = os.environ.get("CAM_DEVICE_PATH", "/dev/video0")
        # Reduced resolution for Pi 3 - match config.toml defaults (512x512)
        self.req_w = int(os.environ.get("CAM_WIDTH", "512"))
        self.req_h = int(os.environ.get("CAM_HEIGHT", "512"))
        self.req_fps = int(os.environ.get("CAM_FPS", "15"))  # Reduced FPS for Pi 3
        # Lower JPEG quality for smaller file sizes - can be overridden with JPEG_QUALITY env var
        self.jpeg_quality = int(os.environ.get("JPEG_QUALITY", "65"))

        # Enable tracking view with SHOW_TRACKING_VIEW=1
        self.show_tracking = os.environ.get("SHOW_TRACKING_VIEW", "0") != "0"
        
        # Tracking state (latest detections)
        self.ball_bbox: Optional[Tuple[float, float, float, float]] = None
        self.laser_pos: Optional[Tuple[float, float]] = None
        self.last_frame_for_display: Optional[np.ndarray] = None

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
        
        # Subscribe to detection topics if tracking view is enabled
        if self.show_tracking:
            self.get_logger().info("Tracking view enabled - subscribing to detection topics...")
            self.ball_sub = self.create_subscription(
                String,
                '/ball_coords',
                self.ball_coords_callback,
                10
            )
            self.laser_sub = self.create_subscription(
                String,
                '/laser_pos',
                self.laser_pos_callback,
                10
            )
            # Create display window
            cv2.namedWindow("Camera Tracking View", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camera Tracking View", 640, 640)
            # Create timer for display update (30 Hz)
            self.display_timer = self.create_timer(1.0/30.0, self.update_display)

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
        self.get_logger().info(f"  Tracking view: {self.show_tracking}")
        self.get_logger().info("")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /image (sensor_msgs/Image, encoding='jpeg', MJPEG compressed)")
        if self.show_tracking:
            self.get_logger().info("")
            self.get_logger().info("Tracking view enabled:")
            self.get_logger().info("  Subscribing to /ball_coords and /laser_pos")
            self.get_logger().info("  Real-time visualization in OpenCV window")
        self.get_logger().info("")
        self.get_logger().info("Press Ctrl+C to stop.")
    
    def ball_coords_callback(self, msg: String) -> None:
        """Callback for ball coordinates"""
        self.ball_bbox = parse_ball_coords(msg.data)
    
    def laser_pos_callback(self, msg: String) -> None:
        """Callback for laser position"""
        self.laser_pos = parse_laser_pos(msg.data)
    
    def update_display(self) -> None:
        """Update the tracking visualization window"""
        if self.last_frame_for_display is None:
            return
        
        # Create a copy to draw on
        display_frame = self.last_frame_for_display.copy()
        
        # Draw ball bounding box (red)
        if self.ball_bbox is not None:
            x1, y1, x2, y2 = self.ball_bbox
            cv2.rectangle(
                display_frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 0, 255),  # Red in BGR
                2
            )
            # Draw center point
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            cv2.circle(display_frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)
            # Label
            cv2.putText(
                display_frame,
                "Ball",
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2
            )
        
        # Draw laser position (green circle)
        if self.laser_pos is not None:
            lx, ly = self.laser_pos
            cv2.circle(display_frame, (int(lx), int(ly)), 8, (0, 255, 0), -1)  # Green in BGR
            # Draw crosshair
            cv2.line(display_frame, (int(lx) - 12, int(ly)), (int(lx) + 12, int(ly)), (0, 255, 0), 2)
            cv2.line(display_frame, (int(lx), int(ly) - 12), (int(lx), int(ly) + 12), (0, 255, 0), 2)
            # Label
            cv2.putText(
                display_frame,
                "Laser",
                (int(lx) + 15, int(ly) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
        
        # Show FPS and frame count
        info_text = f"Frame: {self.frame_count} | FPS: {self.fps:.1f}"
        cv2.putText(
            display_frame,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )
        
        # Display the frame
        cv2.imshow("Camera Tracking View", display_frame)
        cv2.waitKey(1)

    def _sig(self, *_):
        self.get_logger().info("Shutting down...")
        try:
            if self.cap.isOpened():
                self.cap.release()
            if self.show_tracking:
                cv2.destroyAllWindows()
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

        # Save frame for tracking display (if enabled)
        if self.show_tracking:
            self.last_frame_for_display = frame.copy()
        
        # Publish throttling (reduce network overhead)
        self.frame_count += 1
        if self.frame_count % self.publish_every_n != 0:
            return

        # Encode frame as JPEG
        # Note: If camera provides MJPEG directly, we could use it without re-encoding
        # But OpenCV doesn't expose raw MJPEG easily, so we re-encode
        # For better performance, use the Rust version which uses raw MJPEG from camera
        enc_ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not enc_ok or jpeg is None or len(jpeg) < 100:
            self.bad_count += 1
            return

        # Publish to /image topic as sensor_msgs/Image with encoding="jpeg"
        # This matches the Rust version behavior
        msg = Image()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_frame"
        msg.height = h
        msg.width = w
        msg.encoding = "jpeg"  # JPEG compressed data
        msg.is_bigendian = 0
        msg.step = 0  # Step is 0 for compressed formats
        msg.data = jpeg.tobytes()
        self.pub_image.publish(msg)
        
        # Log first few frames and then periodically
        if self.frame_count <= 5 or self.frame_count % 60 == 0:
            self.get_logger().info(
                f"Published frame #{self.frame_count}: {w}x{h} MJPEG ({len(msg.data)} bytes compressed, "
                f"skip={self.publish_every_n}, bad={self.bad_count})"
            )

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
