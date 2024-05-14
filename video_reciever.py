import cv2
import socket
import numpy as np


class VideoStreamReceiver:
    def __init__(self, listen_ip="0.0.0.0", listen_port=5000):
        # Define socket parameters
        # "0.0.0.0"   Listen on all interfaces
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((listen_ip, listen_port))
        # Listen for incoming connections
        self.sock.listen(1)
        # Accept the connection
        self.client_socket, address = self.sock.accept()
        print(f"Connected with {address} {self.client_socket}")

    def receive_data(self):
        data = self.client_socket.recv(1024 * 1024)
        if data:
            return data
        return None

    def send_byte(self, byte):
        # Check for valid key press (replace with your desired key detection method)
        # if key == ord('q'):
        #     return False  # Stop sending key press if 'q' is pressed
        # Encode key press to bytes
        # key_bytes = str(key).encode()
        # Send key press data
        # self.sock.sendall(key_bytes)
        self.sock.sendall(byte)
        return True  # Indicate successful key press sending

    def close(self):
        self.client_socket.close()
        self.sock.close()


if __name__ == '__main__':
    video_stream_receiver = VideoStreamReceiver()

    while True:
        # Receive encoded frame data
        frame_encoded = video_stream_receiver.receive_data()

        if len(frame_encoded) == 0:
            print("Error: Empty frame received, skipping.")
            continue

        try:
            # Convert the encoded frame data to a numpy array
            frame_decoded = np.frombuffer(frame_encoded, dtype=np.uint8)
            # Decode the frame
            frame = cv2.imdecode(frame_decoded, cv2.IMREAD_COLOR)

            if frame is None:
                print("Error: Failed to decode frame!")
                continue

            # Display the received frame
            cv2.imshow('Drone Camera Stream', frame)

            # Check for 'q' key to quit
            key_byte = cv2.waitKey(1)
            if key_byte & 0xFF == ord('q'): #27 Esc
                break
            elif key_byte:
                print(f'Key: {key_byte}')
                video_stream_receiver.send_byte(key_byte)

        except Exception as e:
            print("Error:", e)

    # Close the socket and windows
    video_stream_receiver.close()
    cv2.destroyAllWindows()
