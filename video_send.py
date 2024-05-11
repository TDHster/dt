import cv2
import socket
import numpy as np
import time

# Define camera index (replace with your camera index)
camera_index = 0

# Initialize video capture
cap = cv2.VideoCapture(camera_index)

# Define socket parameters (replace with receiver's IP address)
server_ip = "192.168.1.169"  # Replace with receiver's IP address
server_port = 5000

# Create socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the receiver
sock.connect((server_ip, server_port))

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Resize the frame (optional)
    resized_frame = cv2.resize(frame, (320, 240))  # Reduce resolution

    # Encode the frame using JPEG compression (adjust quality for balance)
    frame_encoded = cv2.imencode('.jpg', resized_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]

    # Send the encoded frame to the receiver
    sock.sendall(frame_encoded)

    # Adjust send interval based on frame rate and network conditions
    time.sleep(0.05)  # Adjust as needed

# Close the socket and video capture when finished
sock.close()
cap.release()