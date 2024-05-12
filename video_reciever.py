import cv2
import socket
import numpy as np


class VideoStreamReceiver:
    def __init__(self, listen_ip="0.0.0.0", listen_port='5000'):
        # Define socket parameters
        # "0.0.0.0"  # Listen on all interfaces
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((listen_ip, listen_port))
        # Listen for incoming connections
        self.sock.listen(1)
        # Accept the connection
        self.client_socket, address = self.sock.accept()
        print(f"Connected with {address} {self.client_socket}")

    def get_data_from_net(self):
        return self.client_socket.recv(1024 * 1024)

    def close(self):
        self.client_socket.close()
        self.sock.close()


if __name__ == '__main__':
    video_stream_receiver = VideoStreamReceiver()

    while True:
        # Receive encoded frame data
        frame_encoded = video_stream_receiver.get_data_from_net()

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
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Exception as e:
            print("Error:", e)

    # Close the socket and windows
    video_stream_receiver.close()
    cv2.destroyAllWindows()
