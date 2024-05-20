#!./venv/bin/python3 drone_main.py

import cv2
from object_tracker import CentroidTracker
import math
import heapq
from video_send import NetworkConnection, get_key_from_byte
from control_drone import MavlinkControl
from object_detector import NeuroNetObjectDetector
from object_detector import filter_by_target_class_id

print('Starting.')
print(f"OpenCV version: {cv2.__version__}")

print(f'Starting connection to mavlink.')
dron_control = MavlinkControl('udpout:127.0.0.1:14550')
# dron_control.arm()

# detection_threshold = 0.45  # Threshold to detect object
detection_threshold = 0.3  # Threshold to detect object

INPUT_VIDEO_WIDTH = 320
INPUT_VIDEO_HEIGHT = 200
# INPUT_VIDEO_WIDTH = 640
# INPUT_VIDEO_HEIGHT = 480
INPUT_VIDEO_FPS = 5


# video_path = 'test_videos/6387-191695740.mp4'  # Commercial from top
# video_path = 'test_videos/188778-883818276_small.mp4' # Two womans
# video_path = 'test_videos/10831-226624994.mp4'  # Square
# cap = cv2.VideoCapture(video_path)

rtsp_url = "rtsp://localhost:8554/cam"
opencv_device = rtsp_url
# opencv_device = 0
cap = cv2.VideoCapture(opencv_device)
if not cap.isOpened():
    print("Error opening video stream or file")
    exit()
print(f'Video device {opencv_device} opened.')

# Dictionary mapping keys to commands
key_to_command = {
    'w': "Move forward",
    's': "Move backward",
    'a': "Move left",
    'd': "Move right",
    'q': "Yaw left",
    'e': "Yaw right",
    ' ': "Select target",
    '\r': "Attack"  # Use '\r' for the enter key
}

# Set the frame width, height, and FPS for the capture object
cap.set(cv2.CAP_PROP_FRAME_WIDTH, INPUT_VIDEO_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, INPUT_VIDEO_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, INPUT_VIDEO_FPS)


def find_nearest_object_id(objects):
    # Initialize an empty priority queue
    pq = []
    # Calculate distances and add them to the priority queue
    for object_id, object_data in objects.items():
        object_x, object_y = object_data[:2]
        distance = math.sqrt((object_x - INPUT_VIDEO_WIDTH/2)**2 + (object_y - INPUT_VIDEO_HEIGHT/2)**2)
        heapq.heappush(pq, (distance, object_id))

    # Extract the nearest object ID from the priority queue (smallest distance)
    nearest_distance, nearest_object_id = heapq.heappop(pq)

    return nearest_object_id


# object_detector = NeuroNetObjectDetector
classFile='neuronet/coco.names'
with open(classFile, 'rt') as f:
    object_class_names = f.read().rstrip('\n').split('\n')

configPath = 'neuronet/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'neuronet/frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)
# enf of object_detector = NeuroNetObjectDetector
print(f'Object NN detector configured.')

object_tracker = CentroidTracker(max_disappeared_frames=50, distance_threshold=50)

receiver_ip = "192.168.0.169"
server_port = 5000
netconnection = NetworkConnection(receiver_ip="192.168.0.169", server_port=5000)
print(f'Network connection started with {receiver_ip}{server_port}.')

target_object_id = None
object_id_near_center = None

print('Starting program main loop.')
while True:
    success, frame = cap.read()
    # Resize the frame to 320x200 while maintaining aspect ratio
    if not success:  # Check success flag
        continue
    frame = cv2.resize(frame, (INPUT_VIDEO_WIDTH, INPUT_VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)

    print(f'{frame.shape=}')

    classIds, confs, bbox = net.detect(frame, confThreshold=detection_threshold)
    print(f'classIds={classIds}, bbox={bbox}')

    classIds, bbox = filter_by_target_class_id(classIds, bbox,
                                               names_list=object_class_names,
                                               target_class_name='person')

    objects = object_tracker.update(bbox)

    print(f'objects={objects}')
    if objects:
        object_id_near_center = find_nearest_object_id(objects)
        for object_id, (x, y, w, h) in objects.items():
            print(f'{object_id, (x, y, w, h)}')
            rect_top_left = (int(x - w / 2), int(y - h / 2))
            rect_bottom_right = (int(x + w / 2), int(y + h / 2))
            if object_id == object_id_near_center:
                # cv2.putText(frame, f'{object_id}', (x - 10, y - 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 255, 255), 2)
            elif object_id == target_object_id:
                cv2.line(frame, (int(INPUT_VIDEO_WIDTH / 2), int(INPUT_VIDEO_HEIGHT / 2)),
                         (x, y), (0, 0, 255), thickness=2)
                yaw_pixels = INPUT_VIDEO_WIDTH/2 - x
                elevation_pixels = INPUT_VIDEO_HEIGHT/2 - y
                cv2.putText(frame, f'Yaw: {yaw_pixels} elev: {elevation_pixels}', (10, 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 2)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 0, 255), 2)
                dron_control.throttle_yaw = (elevation_pixels/INPUT_VIDEO_HEIGHT, yaw_pixels/INPUT_VIDEO_WIDTH)
            else:
                # cv2.putText(frame, f'{object_id}', (x - 10, y - 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 255, 0), 1)

            # cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

    # cv2.imshow("Output", frame)
    netconnection.send_frame(frame)
    # video_stream_sender.make_time_delay(0.05)  # Adjust as needed
    # try:
    #     received_key = netconnection.key_queue.get(timeout=0.1)
    #     command = key_to_command.get(received_key, "Unknown command")
    #     print(f"Received from Queue: {received_key}, '{command}'")
    # except netconnection.key_queue.Empty:
    #     pass  # No data in queue, continue the loop


    # Check for received keys from the queue
    if not netconnection.key_queue.empty():
        try:
            received_key = netconnection.key_queue.get(timeout=0.1)
            command = key_to_command.get(received_key, "Unknown command")
            print(f"Received from Queue: {received_key}, '{command}'")
            if command == 'Select target':
                target_object_id = object_id_near_center
                print(f'Select target: {target_object_id}')
            elif command == 'To target':
                dron_control.to_target()

        except netconnection.key_queue.Empty:
            pass  # No data in queue, continue the loop

    if cv2.waitKey(1) == 27:  # Esc key
        break

cap.release()
netconnection.close()