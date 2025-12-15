#!/usr/bin/env python3
"""
Simple Camera Streamer - Uses only OpenCV (works with standard USB webcams)
Streams MJPEG over HTTP - works with any browser or OpenCV client

Works with:
  - Standard USB webcams (no v4l2 required)
  - Built-in laptop cameras
  - Any camera supported by OpenCV

Run:
  python3 stream_simple.py

Then open browser:
  http://localhost:8080  (or http://raspberry-pi-ip:8080 on network)

Environment variables:
  CAMERA_ID: Camera device ID (default: 0)
  CAMERA_WIDTH: Resolution width (default: 640)
  CAMERA_HEIGHT: Resolution height (default: 480)
  CAMERA_FPS: Target FPS (default: 15)
  JPEG_QUALITY: JPEG quality 1-100 (default: 65)
  HTTP_PORT: HTTP server port (default: 8080)
  PREFER_MJPEG: Set to "0" to disable MJPEG preference (default: "1")
"""

import cv2
import socket
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
import time

# Configuration (can be overridden with environment variables)
CAMERA_ID = int(os.environ.get("CAMERA_ID", "0"))  # Or 1, 2, etc. (standard USB webcam)
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.environ.get("CAMERA_FPS", "15"))
JPEG_QUALITY = int(os.environ.get("JPEG_QUALITY", "65"))
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

# Set to True to prefer MJPEG (hardware encoding), False to use default format
PREFER_MJPEG = os.environ.get("PREFER_MJPEG", "1") != "0"

# Global frame buffer
frame_lock = threading.Lock()
current_frame = None
frame_count = 0

class CameraThread(threading.Thread):
    """Background thread to capture frames"""
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.running = True
        
    def run(self):
        global current_frame, frame_count
        
        print(f"Opening camera {CAMERA_ID} (standard USB webcam)...")
        cap = cv2.VideoCapture(CAMERA_ID)
        
        if not cap.isOpened():
            print(f"ERROR: Failed to open camera {CAMERA_ID}!")
            print("  Try changing CAMERA_ID to 1, 2, etc.")
            return
        
        # Try to set MJPEG format if preferred (many USB webcams support it)
        if PREFER_MJPEG:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # Set resolution and FPS (camera will negotiate closest supported values)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        
        # Get actual negotiated settings
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        # Check format
        fourcc_code = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])
        
        print(f"Camera opened successfully!")
        print(f"  Resolution: {actual_width}x{actual_height}")
        print(f"  FPS: {actual_fps:.1f}")
        print(f"  Format: {fourcc_str}")
        if actual_width != CAMERA_WIDTH or actual_height != CAMERA_HEIGHT:
            print(f"  Note: Camera negotiated {actual_width}x{actual_height} instead of requested {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
        print(f"Streaming on http://{get_ip()}:{HTTP_PORT}")
        print()
        
        # Warm up camera (some USB webcams need a few frames to stabilize)
        for _ in range(5):
            cap.read()
        
        while self.running:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("WARNING: Failed to read frame from camera")
                time.sleep(0.1)
                continue
            
            # Check frame dimensions (some cameras return wrong size initially)
            h, w = frame.shape[:2]
            if w != actual_width or h != actual_height:
                # Resize to expected dimensions if camera changed resolution
                frame = cv2.resize(frame, (actual_width, actual_height))
            
            # Encode as JPEG (works with any camera format)
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            
            if jpeg is None or len(jpeg) == 0:
                print("WARNING: Failed to encode frame as JPEG")
                time.sleep(0.1)
                continue
            
            with frame_lock:
                current_frame = jpeg.tobytes()
                frame_count += 1
            
            # Limit FPS
            time.sleep(1.0 / CAMERA_FPS)
        
        cap.release()

def get_ip():
    """Get local IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"

class StreamHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG stream"""
    
    def log_message(self, format, *args):
        """Suppress access logs"""
        pass
    
    def do_GET(self):
        if self.path == '/':
            # HTML page with embedded video
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = f"""
            <html>
            <head><title>Camera Stream</title></head>
            <body style="margin:0; padding:0; background:#000; display:flex; justify-content:center; align-items:center; height:100vh;">
            <img src="/stream" style="max-width:100%; max-height:100%;">
            </body>
            </html>
            """
            self.wfile.write(html.encode())
        
        elif self.path == '/stream':
            # MJPEG stream
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            try:
                while True:
                    with frame_lock:
                        if current_frame is not None:
                            frame_data = current_frame
                        else:
                            continue
                    
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', len(frame_data))
                    self.end_headers()
                    self.wfile.write(frame_data)
                    self.wfile.write(b'\r\n')
            except:
                pass
        
        elif self.path == '/snapshot':
            # Single JPEG snapshot
            with frame_lock:
                if current_frame is not None:
                    frame_data = current_frame
                else:
                    self.send_error(503, "No frame available")
                    return
            
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Content-length', len(frame_data))
            self.end_headers()
            self.wfile.write(frame_data)
        
        elif self.path == '/status':
            # Status JSON
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            status = f'{{"frames": {frame_count}, "resolution": "{CAMERA_WIDTH}x{CAMERA_HEIGHT}"}}'
            self.wfile.write(status.encode())
        
        else:
            self.send_error(404)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads"""
    pass

def main():
    print("="*60)
    print("Simple Camera Streamer")
    print("="*60)
    print()
    
    # Start camera thread
    camera = CameraThread()
    camera.start()
    
    # Wait for first frame
    time.sleep(2)
    
    # Start HTTP server
    server = ThreadedHTTPServer(('0.0.0.0', HTTP_PORT), StreamHandler)
    
    print(f"HTTP server started on port {HTTP_PORT}")
    print()
    print("Access points:")
    print(f"  Browser:  http://{get_ip()}:{HTTP_PORT}/")
    print(f"  Stream:   http://{get_ip()}:{HTTP_PORT}/stream")
    print(f"  Snapshot: http://{get_ip()}:{HTTP_PORT}/snapshot")
    print(f"  Status:   http://{get_ip()}:{HTTP_PORT}/status")
    print()
    print("Press Ctrl+C to stop")
    print("="*60)
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        camera.running = False
        server.shutdown()

if __name__ == '__main__':
    main()

