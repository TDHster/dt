import cv2
import socket
import numpy as np
import time
import queue
import threading


class NetworkConnection:
    def __init__(self, reciever_ip="192.168.0.169", server_port=5000):
        #queue for recieving data
        self.key_queue = queue.Queue()

        # Define socket parameters (replace with receiver's IP address)
        self.server_ip = reciever_ip  # Replace with receiver's IP address
        self.server_port = server_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((reciever_ip, server_port))  #TODO add try in loop

        # Create threads for receiving data and sending data
        self.receive_thread = threading.Thread(target=self.receive_data, args=(self.sock, self.key_queue,))
        # self.send_thread = threading.Thread(target=self.send_data, args=(1, self.sock,))

        self.receive_thread.start()
        # self.send_thread.start()

    def send_data(self, data, sock):
        # Encode data to bytes
        data_bytes = str(data).encode()
        sock.sendall(data_bytes)

    def send_frame(self, open_cv_frame):
        # frame = cv2.resize(open_cv_frame, (320, 240))  # Reduce resolution
        # Encode the frame using JPEG compression (adjust quality for balance)
        frame_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]
        # Send the encoded frame to the receiver
        self.sock.sendall(frame_encoded)

    def receive_data(self, sock, key_queue):
        # Function to receive data (non-blocking)
        try:
            data = sock.recv(1024).decode()
            if data:
                self.key_queue.put(data)
                print(f"Get from server: {data}")
        except BlockingIOError as e:
            print(f'Error from network receive data: {e}')  # No data received (expected behavior for non-blocking)


    def receive_data_blocking(self, buffer_size=1024):
        # Receive data from the server
        received_data = self.sock.recv(buffer_size)  # Adjust buffer size as needed

        # Check for empty data (connection closed)
        if not received_data:
            print("Empty data. Server disconnected.")
            return

        # Decode received data
        data_decoded = received_data.decode()

        # Print received data
        print(f"Server received: {data_decoded}")
        return data_decoded

    def make_time_delay(self, value_in_seconds=0.05):
        # Adjust send interval based on frame rate and network conditions
        time.sleep(value_in_seconds)

    def close(self):
        self.receive_thread.join()
        # self.send_thread.join()
        self.sock.close()
        print("Network client stopped.")


if __name__ == '__main__':
    camera_index = 0
    cap = cv2.VideoCapture(camera_index)

    netconnection = NetworkConnection(reciever_ip="192.168.0.169", server_port=5000)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        resized_frame = cv2.resize(frame, (320, 240))  # Reduce resolution
        netconnection.send_frame(resized_frame)
        netconnection.make_time_delay(0.05)  # Adjust as needed

        # Check for received keys from the queue
        if not netconnection.key_queue.empty():
            received_key =  netconnection.key_queue.get()
            print(f"Received from Queue: {received_key}")

    netconnection.close()
    cap.release()
