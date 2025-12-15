#!/usr/bin/env python3
"""
Simple Camera Streamer - Uses only OpenCV (no v4l2, no ROS on Pi!)
Streams MJPEG over HTTP - works with any browser or OpenCV client

Run on Raspberry Pi:
  python3 stream_simple.py

Then on PC, open browser:
  http://raspberry-pi-ip:8080

Or use with ROS2 subscriber on PC
"""

import cv2
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
import time

# Configuration
CAMERA_ID = 0  # Or 1, 2, etc.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
JPEG_QUALITY = 65
HTTP_PORT = 8080

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
        
        print(f"Opening camera {CAMERA_ID}...")
        cap = cv2.VideoCapture(CAMERA_ID)
        
        # Try MJPEG first (hardware encoding)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"Camera opened: {actual_width}x{actual_height} @ {actual_fps} FPS")
        print(f"Streaming on http://{get_ip()}:{HTTP_PORT}")
        print()
        
        if not cap.isOpened():
            print("ERROR: Failed to open camera!")
            return
        
        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("WARNING: Failed to read frame")
                time.sleep(0.1)
                continue
            
            # Encode as JPEG
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            
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

