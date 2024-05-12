import cv2
import socket
import numpy as np
import time


class VideoStreamSender:
    def __init__(self, reciever_ip="192.168.0.169", server_port=5000):
        # Define socket parameters (replace with receiver's IP address)
        self.server_ip = reciever_ip  # Replace with receiver's IP address
        self.server_port = server_port

        # Create socket object
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the receiver
        self.sock.connect((reciever_ip, server_port))

    def send_frame(self, open_cv_frame):
        # frame = cv2.resize(open_cv_frame, (320, 240))  # Reduce resolution
        # Encode the frame using JPEG compression (adjust quality for balance)
        frame_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]
        # Send the encoded frame to the receiver
        self.sock.sendall(frame_encoded)

    def make_time_delay(self, value_in_seconds=0.05):
        # Adjust send interval based on frame rate and network conditions
        time.sleep(value_in_seconds)

    def close(self):
        self.sock.close()


if __name__ == '__main__':
    # Define camera index (replace with your camera index)
    camera_index = 0

    # Initialize video capture
    cap = cv2.VideoCapture(camera_index)

    video_sender = VideoStreamSender(reciever_ip="192.168.0.169", server_port=5000)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        resized_frame = cv2.resize(frame, (320, 240))  # Reduce resolution
        video_sender.send_frame(resized_frame)
        video_sender.make_time_delay(0.05)  # Adjust as needed

    video_sender.close()
    cap.release()
