#!./venv/bin/python3 drone_main.py

import cv2
from object_tracker import CentroidTracker
from math import pi, sqrt
import heapq
from video_send import NetworkConnection, get_key_from_byte
# from control_drone import MavlinkJoystickControl as MavlinkControl
# from mavlink_control import MavlinkDrone as MavlinkControl
# from mavlink_th_control import MavlinkDrone as Drone
from mavlink_pwm_control import MavlinkDrone as Drone
# from object_detector import NeuroNetObjectDetector
from object_detector import filter_by_target_class_id
import argparse
from time import sleep

CONTROL_STEP = 0.02
CONTROL_STEP_THRUST = 0.005

INPUT_VIDEO_WIDTH = 320
INPUT_VIDEO_HEIGHT = 200
# INPUT_VIDEO_WIDTH = 640
# INPUT_VIDEO_HEIGHT = 480
#

parser = argparse.ArgumentParser(description="Main drone script")

parser.add_argument(
    "-c", "--camera", type=str, default="0",
    help="Specify path for camera connection ex: 0 for opencv device, for RTSP: rtsp://localhost:8554/cam",
)
parser.add_argument(
    "-fps", type=int, default=5,
    help="FPS for drone camera."
)
parser.add_argument(
    "-m", "--mavlink", type=str, default="udpin:127.0.0.1:14550",
    help="Specify path for mavlink/mavproxy connection.",
)
parser.add_argument(
    "-g", "--groundstation_connection_string", type=str, default="192.168.0.169:5000",
    help="Specify path for mavlink/mavproxy connection.",
)
parser.add_argument(
    "-pidx", type=float, default=0.1, help="PID_X for drone control.", metavar='VALUE'
)
parser.add_argument(
    "-pidz", type=float, default=0.1, help="PID_Z (throttle) for drone control.", metavar='VALUE'
)
parser.add_argument(
    "-pidyaw", type=float, default=0.35, help="PID_YAW for drone control.", metavar='VALUE'
)
parser.add_argument(
    "-dt", "--detection_threshold", type=float, default=0.45, help="detection_threshold for drone control.", metavar='VALUE'
)


args = parser.parse_args()

opencv_device = args.camera
INPUT_VIDEO_FPS = args.fps
mavproxy_connect_string = args.mavlink
gs_connection_string = args.groundstation_connection_string
PID_X = args.pidx
PID_YAW = args.pidyaw
PID_Z = args.pidz
detection_threshold = args.detection_threshold  # 0.3, 0.45
tracker_max_disappeared_frames=50
tracker_distance_threshold=50

print('Starting.')
print(f"Installed OpenCV version: {cv2.__version__}")

print(f'Starting connection to mavlink.')
drone = Drone(mavproxy_connect_string)
# dron.arm()

# video_path = 'test_videos/6387-191695740.mp4'  # Commercial from top
# video_path = 'test_videos/188778-883818276_small.mp4' # Two womans
# video_path = 'test_videos/10831-226624994.mp4'  # Square
# cap = cv2.VideoCapture(video_path)

# rtsp_url = "rtsp://localhost:8554/cam"
# opencv_device = rtsp_url
# opencv_device = 0
try:
    # Try converting to integer
    opencv_device = int(opencv_device)
except ValueError:
    # If conversion fails, return the original string
    pass

print(f'Try to open video device: {opencv_device}')
try:
    cap = cv2.VideoCapture(opencv_device)
except Exception as e:
    print(f'Error opening video source {opencv_device}: {e}')
    exit(1)
if not cap.isOpened():
    print("Error opening video stream or file")
    exit()
print(f'Video device {opencv_device} opened.')

# Dictionary mapping keys to commands
key_to_command = {
    't': "Takeoff",
    'g': "Land",
    'w': "Move forward",
    's': "Move backward",
    'a': "Move left",
    'd': "Move right",
    'q': "Yaw left",
    'e': "Yaw right",
    'p': "Move up",
    'l': "Move down",
    ' ': "Select target",
    'с': "Clear target",
    '\r': "To target"  # Use '\r' for the enter key
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
        distance = sqrt((object_x - INPUT_VIDEO_WIDTH/2)**2 + (object_y - INPUT_VIDEO_HEIGHT/2)**2)
        heapq.heappush(pq, (distance, object_id))

    # Extract the nearest object ID from the priority queue (smallest distance)
    nearest_distance, nearest_object_id = heapq.heappop(pq)

    return nearest_object_id


class ObjectDetector:
    def __init__(self):
        # object_detector = NeuroNetObjectDetector
        objects_class_names = 'neuronet/coco.names'
        with open(objects_class_names, 'rt') as f:
            self.object_class_names = f.read().rstrip('\n').split('\n')

        config_path = 'neuronet/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        weights_path = 'neuronet/frozen_inference_graph.pb'

        self.net = cv2.dnn_DetectionModel(weights_path, config_path)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)
        print(f'Object NN detector configured.')
        # enf of object_detector = NeuroNetObjectDetector

    def detect(self, frame, detection_threshold=0.5):
        classIds, confs, bbox = self.net.detect(frame, confThreshold=detection_threshold)
        return classIds, confs, bbox


    def filter(self, class_ids, bbox, target_class_name='person'):
        if len(class_ids):
            # Efficient filtering using boolean indexing
            # keep_indices = classIds == object_class_names.index(target_class_name) + 1  # Indices where specified class ID (1 is person)
            keep_indices = class_ids == self.object_class_names.index(
                target_class_name) + 1  # Indices where specified class ID (1 is person)
            # keep_indices = classIds == 1  # Indices where specified class ID (1 is person)
            class_ids = class_ids[keep_indices]
            bbox = bbox[keep_indices]
            return class_ids, bbox
        return (), ()


object_detector = ObjectDetector()

object_tracker = CentroidTracker(max_disappeared_frames=tracker_max_disappeared_frames, distance_threshold=tracker_distance_threshold)

print(f'Trying connect with ground station {gs_connection_string}.')
netconnection = NetworkConnection(gs_connection_string=gs_connection_string)
print(f'Network connection established.')

target_object_id = None
object_id_near_center = None
target_object_diagonal = None
need_reset_yaw = True

print('Starting program main loop.')
while True:
    success, frame = cap.read()
    if not success:  # Check success flag
        continue
    frame = cv2.resize(frame, (INPUT_VIDEO_WIDTH, INPUT_VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)
    # Resize the frame to 320x200 while maintaining aspect ratio
    # print(f'{frame.shape=}')
    # classIds, confs, bbox = net.detect(frame, confThreshold=detection_threshold)  # moved to class
    classIds, confs, bbox = object_detector.detect(frame, detection_threshold=detection_threshold)

    # print(f'classIds={classIds}, bbox={bbox}')
    if classIds is None:
        # Handle the case where no target class IDs were found
        print("No target class IDs detected")
        continue

    # classIds, bbox = filter_by_target_class_id(classIds, bbox,
    #                                            names_list=object_class_names,
    #                                            target_class_name='person')
    classIds, bbox = object_detector.filter(classIds, bbox, target_class_name='person')

    objects = object_tracker.update(bbox)

    if (target_object_id not in objects) and not need_reset_yaw:
        drone.yaw = 0
        need_reset_yaw = False
        print(f'Yaw 0, because object lost')

    # print(f'objects={objects}')
    if objects:
        object_id_near_center = find_nearest_object_id(objects)
        target_found = False

        for object_id, (x, y, w, h) in objects.items():
            # print(f'{object_id, (x, y, w, h)}')
            rect_top_left = (int(x - w / 2), int(y - h / 2))
            rect_bottom_right = (int(x + w / 2), int(y + h / 2))
            if object_id == target_object_id:
                need_reset_yaw = True

                cv2.line(frame, (int(INPUT_VIDEO_WIDTH / 2), int(INPUT_VIDEO_HEIGHT / 2)),
                         (x, y), (0, 0, 255), thickness=2)
                yaw_pixels = x - INPUT_VIDEO_WIDTH/2
                elevation_pixels = INPUT_VIDEO_HEIGHT/2 - y
                cv2.putText(frame, f'Yaw: {yaw_pixels} elev: {elevation_pixels}', (10, 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 2)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 0, 255), 2)
                target_object_current_diagonal = sqrt(w*w + h*h)
                if not target_object_diagonal:
                    target_object_diagonal = target_object_current_diagonal
                dx = (target_object_diagonal - target_object_current_diagonal) * PID_X
                # drone.pitch = dx * PID_X
                print(f'Sending yaw: {yaw_pixels/INPUT_VIDEO_WIDTH * PID_YAW}')
                drone.yaw = yaw_pixels/INPUT_VIDEO_WIDTH * PID_YAW  # need correction factor  *diagonal/image_diagonal
                dz = elevation_pixels/INPUT_VIDEO_HEIGHT * PID_Z
                # drone.thrust = dz

            elif object_id == object_id_near_center:
                # cv2.putText(frame, f'{object_id}', (x - 10, y - 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 255, 255), 2)
            else:
                # cv2.putText(frame, f'{object_id}', (x - 10, y - 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 255, 0), 1)

            # cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)


    # cv2.imshow("Output", frame)
    try:
        netconnection.send_frame(frame)
    except BrokenPipeError as e:
        print(f'Error sending frame to ground station {e} (receiver program running?)')
    # video_stream_sender.make_time_delay(0.05)  # Adjust as needed
    # try:
    #     received_key = netconnection.key_queue.get(timeout=0.1)
    #     command = key_to_command.get(received_key, "Unknown command")
    #     print(f"Received from Queue: {received_key}, '{command}'")
    # except netconnection.key_queue.Empty:
    #     pass  # No data in queue, continue the loop

    # target_object_id = object_id_near_center #  for debug - autotarget nearest
    # Check for received keys from the queue
    if not netconnection.key_queue.empty():
        try:
            received_byte = netconnection.key_queue.get(timeout=0.1)
            key = chr(received_byte)
            key_encoded = key.encode()
            print(f'{key=}\t{key_encoded=}')
            command = key_to_command.get(key, "Unknown command")
            print(f"Received from Queue: {key}, '{command}'")
            if command == 'Select target':
                target_object_id = object_id_near_center
                print(f'Select target: {target_object_id}')
                target_object_diagonal = None
            elif command == 'To target':
                drone.to_target()
            elif command == "Clear target":
                target_object_id = None
            elif command == "Yaw left":
                drone.yaw = drone.yaw - CONTROL_STEP
            elif command == "Yaw right":
                drone.yaw = drone.yaw + CONTROL_STEP
            elif command == "Move up":
                drone.thrust = drone.thrust + CONTROL_STEP_THRUST
            elif command == "Move down":
                drone.thrust = drone.thrust - CONTROL_STEP_THRUST
            elif command == "Move forward":
                drone.pitch = drone.pitch + CONTROL_STEP
            elif command == "Move backward":
                drone.pitch = drone.pitch - CONTROL_STEP
            elif command == "Move left":
                drone.roll = drone.roll - CONTROL_STEP
            elif command == "Move right":
                drone.roll = drone.roll + CONTROL_STEP
            elif command == "Land":
                drone.manual_land()
            elif command == "Takeoff":
                drone.manual_takeoff()
            else:
                print(f'{command=} not known')

        except netconnection.key_queue.Empty:
            pass  # No data in queue, continue the loop
    print(f'{drone.thrust=}\t{drone.pitch=}\t{drone.roll=}\t{drone.yaw=}')
    if cv2.waitKey(1) == 27:  # Esc key
        break

cap.release()
netconnection.close()
drone.manual_land()
drone.mode_land()
drone.disarm()
drone.emergency_stop()
# drone.manual_land()
# drone.mode_land()