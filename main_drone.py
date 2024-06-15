#!./venv/bin/python3 drone_main.py

import cv2
from object_tracker import CentroidTracker
from math import pi, sqrt, sin, cos, atan, tan, degrees, radians
import heapq
from video_send import NetworkConnection, get_key_from_byte
# from control_drone import MavlinkJoystickControl as MavlinkControl
# from mavlink_control import MavlinkDrone as MavlinkControl
# from mavlink_th_control import MavlinkDrone as Drone
# from mavlink_pwm_control import MavlinkDrone as Drone # was almost good
from mavlink_drone import MavlinkDrone as Drone
# from mavsdk_control import MavlinkDrone as Drone

# from object_detector import NeuroNetObjectDetector
# from object_detector import filter_by_target_class_id
from object_detector import ObjectDetector
import argparse
from time import sleep
from bcolors import bcolors

YAW_STEP = 30  # degrees
MOVEMENT_STEP = 0.5  # meters

TARGET_OBJECT_HEIGHT = 2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_DIAGONAL_FOV = 78  # degrees for logitech c920
diagonal_fov_rad = radians(CAMERA_DIAGONAL_FOV)
CAMERA_VERTICAL_FOV = degrees(2 * atan(tan(diagonal_fov_rad / 2) / sqrt(1 + (CAMERA_WIDTH / CAMERA_HEIGHT) ** 2)))
CAMERA_HORIZONTAL_FOV = degrees(2 * atan((CAMERA_WIDTH / CAMERA_HEIGHT) * tan(radians(CAMERA_VERTICAL_FOV) / 2)))

# 70.2x39.6
camera_gimbal_pitch_angle = 0  # alight to forward
print(f'Using setup for camera: {CAMERA_DIAGONAL_FOV=}, {CAMERA_HORIZONTAL_FOV=}, {CAMERA_VERTICAL_FOV=}')

INPUT_VIDEO_WIDTH = 320
INPUT_VIDEO_HEIGHT = 200
# INPUT_VIDEO_WIDTH = 640
# INPUT_VIDEO_HEIGHT = 480

HORIZONTAL_ANGLE_PER_PIXEL = CAMERA_HORIZONTAL_FOV / INPUT_VIDEO_WIDTH
VERTICAL_ANGLE_PER_PIXEL = CAMERA_VERTICAL_FOV / INPUT_VIDEO_HEIGHT

# TARGET_OBJECT_SIZE_PERCENT_OF_IMAGE_HEIGHT = 0.7
# TARGET_OBJECT_DIAGONAL = INPUT_VIDEO_HEIGHT * TARGET_OBJECT_SIZE_PERCENT_OF_IMAGE_HEIGHT * cos(30 * (pi / 180))
DESIRED_OBJECT_DISTANCE = 4

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
# 0.3 0.1 0.05 new formula 0.2 0.6 0.4 0.5
parser.add_argument(
    "-pidx", type=float, default=0.4, help="PID_X for drone control.", metavar='VALUE'
)
# 0.5
parser.add_argument(
    "-pidz", type=float, default=0.5, help="PID_Z (throttle) for drone control.", metavar='VALUE'
)
# 1 1.5
parser.add_argument(
    "-pidyaw", type=float, default=1.1, help="PID_YAW for drone control.", metavar='VALUE'
)
parser.add_argument(
    "-dt", "--detection_threshold", type=float, default=0.45, help="detection_threshold for drone control.",
    metavar='VALUE'
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
tracker_max_disappeared_frames = 50
tracker_distance_threshold = 80

target_object_height = 2.5

print('Starting.')
print(f"Installed OpenCV version: {cv2.__version__}")

print(f'Starting connection to mavlink.')
drone = Drone(mavproxy_connect_string)

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
        distance = sqrt((object_x - INPUT_VIDEO_WIDTH / 2) ** 2 + (object_y - INPUT_VIDEO_HEIGHT / 2) ** 2)
        heapq.heappush(pq, (distance, object_id))

    # Extract the nearest object ID from the priority queue (smallest distance)
    nearest_distance, nearest_object_id = heapq.heappop(pq)

    return nearest_object_id


object_detector = ObjectDetector()

object_tracker = CentroidTracker(
    max_disappeared_frames=tracker_max_disappeared_frames,
    distance_threshold=tracker_distance_threshold
)

print(f'Trying connect with ground station {gs_connection_string}.')
netconnection = NetworkConnection(gs_connection_string=gs_connection_string)
print(f'Network connection established.')

target_object_id = None
object_id_near_center = None
target_object_diagonal = None
need_reset_movement_if_lost = False

print('Starting program main loop.')
while True:
    success, frame = cap.read()
    if not success:  # Check success flag
        continue
    frame = cv2.resize(frame, (INPUT_VIDEO_WIDTH, INPUT_VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # convert to black and white  # dont work with neurodetect
    # Resize the frame to 320x200 while maintaining aspect ratio

    classIds, confs, bbox = object_detector.detect(frame, detection_threshold=detection_threshold)
    # print(f'classIds={classIds}, bbox={bbox}')

    if classIds is None:
        # Handle the case where no target class IDs were found
        print("No target class IDs detected")
        continue

    # classIds, bbox = object_detector.filter(classIds, bbox, target_class_name='person')
    classIds, bbox = object_detector.filter(classIds, bbox, target_class_names=('person', 'car'))

    objects = object_tracker.update(bbox)

    if ((target_object_id not in objects) and  # object lost
            need_reset_movement_if_lost and
            drone.to_target_status() == False):
        drone.change_position(0, 0, 0)
        drone.yaw(0)
        need_reset_movement_if_lost = False
        print(f'{bcolors.FAIL}All sticks to zero, object lost{bcolors.ENDC}')

    # print(f'objects={objects}')
    if objects:
        object_id_near_center = find_nearest_object_id(objects)
        target_found = False

        for object_id, (x, y, w, h) in objects.items():
            # print(f'{object_id, (x, y, w, h)}')
            rect_top_left = (int(x - w / 2), int(y - h / 2))
            rect_bottom_right = (int(x + w / 2), int(y + h / 2))
            if object_id == target_object_id:
                need_reset_movement_if_lost = True

                cv2.line(frame, (int(INPUT_VIDEO_WIDTH / 2), int(INPUT_VIDEO_HEIGHT / 2)),
                         (x, y), (0, 0, 255), thickness=2)
                yaw_pixels = x - INPUT_VIDEO_WIDTH / 2
                yaw_angle = yaw_pixels * HORIZONTAL_ANGLE_PER_PIXEL
                elevation_pixels = INPUT_VIDEO_HEIGHT / 2 - y  # center point
                elevation_angle = elevation_pixels * VERTICAL_ANGLE_PER_PIXEL

                # elevation_pixels = INPUT_VIDEO_HEIGHT/2 - y + 100 # center point shift
                # elevation_pixels = INPUT_VIDEO_HEIGHT;2/3 - y  # add shift up
                # cv2.putText(frame, f'Yaw: {yaw_pixels} elev: {elevation_pixels}', (10, 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 2)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 0, 255), 2)
                target_object_current_diagonal = sqrt(w * w + h * h)
                target_object_distance_approximate = TARGET_OBJECT_HEIGHT/2 / sin(target_object_current_diagonal) + 0.00001  # TODO gimbal pitch angle correcteion needed
                print(f'{bcolors.OKBLUE}Approx distance to object: {target_object_distance_approximate:.2f}{bcolors.ENDC}')
                # dx = ((TARGET_OBJECT_DIAGONAL / target_object_current_diagonal) - 1) * PID_X

                dx = (DESIRED_OBJECT_DISTANCE - target_object_distance_approximate) * PID_X
                dyaw = yaw_angle * PID_YAW # * sin(target_object_distance_approximate)
                # dz = sin(elevation_angle) * PID_Z * 1/sin(target_object_distance_approximate) + 0.001
                # dz = sin(elevation_angle) * PID_Z #  * 1/sin(target_object_distance_approximate) + 0.001
                dz = elevation_angle * 1/20

                print(f'{dx=:.1f}\t{dz=:.1f}\t{dyaw=:.1f}')
                drone.change_position(0, 0, dz)
                print(f'{bcolors.OKCYAN}Sending {dyaw=}{bcolors.ENDC}')
                drone.yaw(yaw=dyaw)

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
            print(f"Received from Queue: {key=}, '{command}', {key_encoded=}")
            if command == 'Select target':
                target_object_id = object_id_near_center
                print(f'Select target: {target_object_id}')
                target_object_diagonal = None
            elif command == 'To target':
                drone.to_target()
            elif command == "Clear target":
                target_object_id = -1
                drone.to_target(False)
            elif command == "Yaw left":
                drone.yaw(yaw=-YAW_STEP)
            elif command == "Yaw right":
                drone.yaw(yaw=YAW_STEP)
            elif command == "Move up":
                drone.change_position(dz=MOVEMENT_STEP)
            elif command == "Move down":
                drone.change_position(dz=-MOVEMENT_STEP)
            elif command == "Move forward":
                drone.change_position(dx=MOVEMENT_STEP)
            elif command == "Move backward":
                drone.change_position(dx=-MOVEMENT_STEP)
            elif command == "Move left":
                drone.change_position(dy=-MOVEMENT_STEP)
            elif command == "Move right":
                drone.change_position(dy=MOVEMENT_STEP)
            elif command == "Land":
                drone.set_mode_land()
                # drone.set_mode_return_to_land()
            elif command == "Takeoff":
                drone.takeoff(2)
            else:
                print(f'{command=} not known')

        except netconnection.key_queue.Empty:
            pass  # No data in queue, continue the loop
    # print(f'{drone.thrust=:0.1f}\t{drone.pitch=:0.1f}\t{drone.roll=:0.1f}\t{drone.yaw=:0.1f}')
    if cv2.waitKey(1) == 27:  # Esc key
        print('Exit main program loop.')
        break

# drone.manual_land()
drone.mode_land()
drone.disarm()
drone.emergency_stop()
cap.release()
netconnection.close()
# drone.manual_land()
# drone.mode_land()
