import cv2
from object_tracker import CentroidTracker
import math
import heapq
import queue


video_path = 'test_videos/6387-191695740.mp4'  # Commercial from top
# video_path = 'test_videos/188778-883818276_small.mp4' # Two womans
# video_path = 'test_videos/10831-226624994.mp4'  # Square
cap = cv2.VideoCapture(video_path)
# From video device
# cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error opening video stream or file")
    exit()

# detection_threshold = 0.45  # Threshold to detect object
detection_threshold = 0.3  # Threshold to detect object


INPUT_VIDEO_WIDTH = 320
INPUT_VIDEO_HEIGHT = 200
INPUT_VIDEO_FPS = 5

# Set the frame width, height, and FPS for the capture object
cap.set(cv2.CAP_PROP_FRAME_WIDTH, INPUT_VIDEO_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, INPUT_VIDEO_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, INPUT_VIDEO_FPS)


def filter_by_target_class_id(classIds, bbox, names_list, target_class_name='person'):
    if len(classIds):
        # Efficient filtering using boolean indexing
        # keep_indices = classIds == object_class_names.index(target_class_name) + 1  # Indices where specified class ID (1 is person)
        keep_indices = classIds == names_list.index(target_class_name) + 1  # Indices where specified class ID (1 is person)
        # keep_indices = classIds == 1  # Indices where specified class ID (1 is person)
        classIds = classIds[keep_indices]
        bbox = bbox[keep_indices]
        return classIds, bbox


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

class NeuroNetObjectDetector:
    def __init__(self):
        self._load_object_class_names()
        configPath = 'neuronet/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        weightsPath = 'neuronet/frozen_inference_graph.pb'

        self.net = cv2.dnn_DetectionModel(weightsPath, configPath)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

    def _load_object_class_names(self, classFile='neuronet/coco.names'):
        self._object_class_names = []
        with open(classFile, 'rt') as f:
            self._object_class_names = f.read().rstrip('\n').split('\n')

    @property
    def object_class_names(self):
        return self._object_class_names

    def detect(self, frame, confThreshold=detection_threshold):
        classIds, confs, bbox = self.net.detect(frame, confThreshold=detection_threshold)
        return classIds, confs, bbox

object_detector = NeuroNetObjectDetector
object_tracker = CentroidTracker(max_disappeared_frames=40, distance_threshold=50)

from video_send import NetworkConnection
netconnection = NetworkConnection()
print(f'Video streamer activated.')

while True:
    success, frame = cap.read()
    # Resize the frame to 320x200 while maintaining aspect ratio
    frame = cv2.resize(frame, (320, 200), interpolation=cv2.INTER_AREA)

    classIds, confs, bbox = object_detector.detect(frame, confThreshold=detection_threshold)
    print(f'{classIds=}, {bbox=}')

    classIds, bbox = filter_by_target_class_id(classIds, bbox,
                                               names_list=object_detector.object_class_names,
                                               target_class_name='person')

    objects = object_tracker.update(bbox)

    print(f'{objects=}')
    if objects:
        object_id_near_center = find_nearest_object_id(objects)
        for object_id, (x, y, w, h) in objects.items():
            print(f'{object_id, (x, y, w, h)}')
            rect_top_left = (int(x - w / 2), int(y - h / 2))
            rect_bottom_right = (int(x + w / 2), int(y + h / 2))
            if object_id == object_id_near_center:
                cv2.putText(frame, f'{object_id}', (x - 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 0, 255), 2)  # Blue rectangle

                cv2.line(frame, (int(INPUT_VIDEO_WIDTH/2), int(INPUT_VIDEO_HEIGHT/2)), (x, y), (0, 0, 255), thickness=2)
                yaw_pixels = INPUT_VIDEO_WIDTH/2 - x
                elevation_pixels = INPUT_VIDEO_HEIGHT/2 - y
                cv2.putText(frame, f'Yaw: {yaw_pixels} elev: {elevation_pixels}', (10, 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 2)

            else:
                cv2.putText(frame, f'{object_id}', (x - 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.rectangle(frame, rect_top_left, rect_bottom_right, (0, 255, 0), 1)  # Blue rectangle

            # cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

    # cv2.imshow("Output", frame)
    netconnection.send_frame(frame)
    # video_stream_sender.make_time_delay(0.05)  # Adjust as needed
    # Check for received keys from the queue
    if not netconnection.key_queue.empty():
        received_key = netconnection.key_queue.get()
        print(f"Received from Queue: {received_key}")

    if cv2.waitKey(1) == 27:  # Esc key
        break
