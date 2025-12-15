#!/usr/bin/env python3
"""
Video Stream Receiver (B.py)
Receives video stream over network using ZeroMQ SUB socket.
Displays frames in OpenCV window.

Usage:
    python B.py [--sender-ip IP] [--port PORT]

Environment variables:
    SENDER_IP: IP address of sender (default: localhost)
    ZMQ_PORT: Port to connect to (default: 5555)
"""

import zmq
import cv2
import numpy as np
import base64
import argparse
import sys
import os
from typing import Optional


def main():
    parser = argparse.ArgumentParser(description='Video stream receiver using ZeroMQ')
    parser.add_argument('--sender-ip', type=str, default=os.environ.get('SENDER_IP', 'localhost'),
                       help='IP address of sender (default: localhost)')
    parser.add_argument('--port', type=int, default=int(os.environ.get('ZMQ_PORT', '5555')),
                       help='Port to connect to (default: 5555)')
    parser.add_argument('--window-name', type=str, default='Receiver',
                       help='Window name for display (default: Receiver)')
    
    args = parser.parse_args()
    
    # Initialize ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    
    connect_address = f"tcp://{args.sender_ip}:{args.port}"
    print(f"Connecting to {connect_address}...")
    
    try:
        socket.connect(connect_address)
    except zmq.ZMQError as e:
        print(f"Error connecting to {connect_address}: {e}")
        print("Make sure:")
        print("  1. The sender is running")
        print("  2. The IP address is correct")
        print("  3. The port matches the sender's port")
        print("  4. Firewall allows the connection")
        sys.exit(1)
    
    # Subscribe to all messages (empty string means subscribe to all)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')
    
    print(f"Connected successfully. Waiting for frames...")
    print("Press 'q' to quit")
    
    frame_count = 0
    last_fps_time = None
    last_frame_time = None
    
    try:
        while True:
            # Receive frame (blocking)
            try:
                jpg_as_text = socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                # No data available, continue
                import time
                time.sleep(0.01)
                continue
            
            # Decode base64
            try:
                jpg_original = base64.b64decode(jpg_as_text)
            except Exception as e:
                print(f"Error decoding base64: {e}")
                continue
            
            # Convert to numpy array
            try:
                jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
            except Exception as e:
                print(f"Error creating numpy array: {e}")
                continue
            
            # Decode JPEG
            try:
                frame = cv2.imdecode(jpg_as_np, flags=cv2.IMREAD_COLOR)
            except Exception as e:
                print(f"Error decoding JPEG: {e}")
                continue
            
            if frame is None:
                print("Warning: Failed to decode frame")
                continue
            
            # Display frame
            cv2.imshow(args.window_name, frame)
            
            frame_count += 1
            
            # Print FPS every second
            import time
            current_time = time.time()
            if last_fps_time is None:
                last_fps_time = current_time
            elif current_time - last_fps_time >= 1.0:
                actual_fps = frame_count / (current_time - last_fps_time)
                latency = ""
                if last_frame_time:
                    avg_latency = (current_time - last_frame_time) * 1000
                    latency = f" | Latency: {avg_latency:.1f}ms"
                print(f"FPS: {actual_fps:.1f} | Frames received: {frame_count}{latency}")
                frame_count = 0
                last_fps_time = current_time
            
            last_frame_time = current_time
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cv2.destroyAllWindows()
        socket.close()
        context.term()
        print("Receiver stopped")


if __name__ == '__main__':
    main()

