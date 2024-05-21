import cv2
import socket
import numpy as np
import time
import queue
import threading


def get_key_from_byte(key_code):
    # Basic mapping dictionary for common special keys
    key_map = {
        27: 'Esc',  # Escape key
        8: 'Backspace',
        9: 'Tab',
        13: 'Enter',
        16: 'Shift',
        17: 'Ctrl',
        18: 'Alt',
        24: 'Home',
        27: 'End',
        33: 'PageUp',
        34: 'PageDown',
        35: 'End',
        36: 'Home',
        37: 'Left',
        38: 'Up',
        39: 'Right',
        40: 'Down',
        45: 'Insert',
        46: 'Delete'
    }

    # Check if ASCII range
    if 32 <= key_code <= 127:
        character = chr(key_code)
    else:
        # Handle special keys using the mapping dictionary
        character = key_map.get(key_code, f'Unknown key ({key_code})')

    return character


class NetworkConnection:
    def __init__(self, receiver_ip="192.168.0.169", server_port=5000):
        # queue for receiving data
        self.key_queue = queue.Queue()

        # Define socket parameters (replace with receiver's IP address)
        self.server_ip = receiver_ip  # Replace with receiver's IP address
        self.server_port = server_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((receiver_ip, server_port))  # TODO add try in loop

        # Create threads for receiving data and sending data
        self.receive_thread = threading.Thread(target=self.receive_data, args=(self.sock, self.key_queue,))
        # self.send_thread = threading.Thread(target=self.send_data, args=(1, self.sock,))

        self.receive_thread.start()
        # self.send_thread.start()

    def _send_data(self, data):
        # Encode data to bytes
        # data_encoded = str(data).encode()
        self.sock.sendall(data)

    def send_frame(self, open_cv_frame):
        # frame = cv2.resize(open_cv_frame, (320, 240))  # Reduce resolution
        # Encode the frame using JPEG compression (adjust quality for balance)
        frame_encoded = cv2.imencode('.jpg', open_cv_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]
        # Send the encoded frame to the receiver
        self._send_data(frame_encoded)

    # def receive_data(self, sock, key_queue, buffer_size=1024):
    #     # Function to receive data (non-blocking)
    #     try:
    #         data = sock.recv(buffer_size).decode()
    #         if data:
    #             self.key_queue.put(data)
    #             print(f"Get from server: {data}")
    #     except BlockingIOError as e:
    #         print(f'Error from network receive data: {e}')  # No data received (expected behavior for non-blocking)

    def receive_data(self, sock, key_queue, buffer_size=1024):
        """Receives data from the socket and adds it to the key queue.

        Args:
            sock: The socket object for receiving data.
            key_queue: A queue to store received key codes.
            buffer_size: The maximum size of data to receive at once.
        """
        while True:
            try:
                data = sock.recv(buffer_size)  # Receive data as bytes
                if data:
                    # Extract and add key code (assuming single byte)
                    key_code = int.from_bytes(data, byteorder='big')
                    print(f"In thread: get from sender: {key_code}")
                    self.key_queue.put(key_code)
                    print(f"In thread: putted in queue.")
            except BlockingIOError as e:
                # No data received (expected for non-blocking)
                pass
            self.make_time_delay()

    def receive_data_blocking(self, buffer_size=1024):
        # Receive data from the server
        received_data = self.sock.recv(buffer_size)  # Adjust buffer size as needed

        # Check for empty data (connection closed)
        if not received_data:
            print("Empty data. Server disconnected.")
            return
        data_decoded = received_data.decode()
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

    netconnection = NetworkConnection(receiver_ip="192.168.0.169", server_port=5000)

    while True:
        try:
            ret, frame = cap.read()
            resized_frame = cv2.resize(frame, (320, 240))  # Reduce resolution
            netconnection.send_frame(resized_frame)
            netconnection.make_time_delay(0.05)  # Adjust as needed

            # Check for received keys from the queue
            if not netconnection.key_queue.empty():
                received_key =  netconnection.key_queue.get()
                print(f"Received from Queue: {received_key}")
        except KeyboardInterrupt:
            break

    cap.release()
    netconnection.close()
