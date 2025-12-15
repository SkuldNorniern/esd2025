#!/usr/bin/env python3
"""
Video Stream Sender (A.py)
Streams video from camera over network using ZeroMQ PUB socket.
Sends frames as JPEG-encoded base64 strings.

Usage:
    python A.py [--camera CAMERA_ID] [--port PORT] [--address ADDRESS]

Environment variables:
    CAMERA_ID: Camera device ID (default: 0)
    ZMQ_PORT: Port to bind to (default: 5555)
    ZMQ_ADDRESS: Address to bind to (default: *)
"""

import cv2
import zmq
import base64
import argparse
import sys
import os
from typing import Optional


def main():
    parser = argparse.ArgumentParser(description='Video stream sender using ZeroMQ')
    parser.add_argument('--camera', type=int, default=int(os.environ.get('CAMERA_ID', '0')),
                       help='Camera device ID (default: 0)')
    parser.add_argument('--port', type=int, default=int(os.environ.get('ZMQ_PORT', '5555')),
                       help='Port to bind to (default: 5555)')
    parser.add_argument('--address', type=str, default=os.environ.get('ZMQ_ADDRESS', '*'),
                       help='Address to bind to (default: *)')
    parser.add_argument('--quality', type=int, default=85, choices=range(1, 101),
                       help='JPEG quality 1-100 (default: 85)')
    parser.add_argument('--fps', type=int, default=None,
                       help='Target FPS (default: camera max)')
    
    args = parser.parse_args()
    
    # Initialize ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    
    bind_address = f"tcp://{args.address}:{args.port}"
    print(f"Binding to {bind_address}...")
    
    try:
        socket.bind(bind_address)
    except zmq.ZMQError as e:
        print(f"Error binding to {bind_address}: {e}")
        print("Make sure the port is not already in use and you have permission to bind.")
        sys.exit(1)
    
    print(f"Socket bound successfully. Waiting for connections...")
    
    # Open camera
    print(f"Opening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        sys.exit(1)
    
    # Set camera properties if specified
    if args.fps:
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Camera opened: {width}x{height} @ {fps:.1f} FPS")
    
    # JPEG encoding parameters
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    
    frame_count = 0
    last_fps_time = None
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Warning: Failed to read frame from camera")
                continue
            
            # Encode frame as JPEG
            success, buffer = cv2.imencode('.jpg', frame, encode_params)
            
            if not success:
                print("Warning: Failed to encode frame as JPEG")
                continue
            
            # Convert to base64
            jpg_as_text = base64.b64encode(buffer)
            
            # Send frame
            try:
                socket.send(jpg_as_text, zmq.NOBLOCK)
            except zmq.Again:
                # No subscribers connected, skip sending
                pass
            
            frame_count += 1
            
            # Print FPS every second
            import time
            current_time = time.time()
            if last_fps_time is None:
                last_fps_time = current_time
            elif current_time - last_fps_time >= 1.0:
                actual_fps = frame_count / (current_time - last_fps_time)
                print(f"FPS: {actual_fps:.1f} | Frames sent: {frame_count}")
                frame_count = 0
                last_fps_time = current_time
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cap.release()
        socket.close()
        context.term()
        print("Sender stopped")


if __name__ == '__main__':
    main()

