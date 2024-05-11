import cv2
import socket
import numpy as np

# Define socket parameters
server_ip = "0.0.0.0"  # Listen on all interfaces
server_port = 5000

# Create socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
sock.bind((server_ip, server_port))

# Listen for incoming connections
sock.listen(1)

# Accept the connection
client_socket, address = sock.accept()

print(f"Connected with {address}")

while True:
    # Receive encoded frame data
    frame_encoded = client_socket.recv(1024 * 1024)

    # Check if the connection is closed
    if not frame_encoded:
        break

    if not (len(frame_encoded) > 0):
        print("Error: Empty frame received!")
        continue

    try:
        # Convert the encoded frame data to a numpy array
        frame_decoded = np.frombuffer(frame_encoded, dtype=np.uint8)

        # Decode the frame
        frame = cv2.imdecode(frame_decoded, cv2.IMREAD_COLOR)

        # Check if the frame is valid
        if frame is None:
            print("Error: Failed to decode frame!")
            continue

        # Display the received frame
        cv2.imshow('Camera Stream', frame)

        # Check for 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except Exception as e:
        print("Error:", e)

# Close the socket and windows
client_socket.close()
sock.close()
cv2.destroyAllWindows()
