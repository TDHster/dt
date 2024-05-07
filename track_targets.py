import cv2
import numpy as np
from object_tracker import MOSSETracker as ObjectTracker

# Загружаем каскадную модель Хаара
# https://github.com/opencv/opencv/tree/master/data/haarcascades
# cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
# cascade = cv2.CascadeClassifier('neuronet/haarcascade_fullbody.xml')


# video_path = 'test_videos/6387-191695740.mp4'  # Commercial from top
video_path = 'test_videos/188778-883818276_small.mp4' # Two womans
# video_path = 'test_videos/10831-226624994.mp4'  # Square
cap = cv2.VideoCapture(video_path)
# From video device
# cap = cv2.VideoCapture(0)

detection_threshold = 0.45  # Threshold to detect object
# detection_threshold = 0.3  # Threshold to detect object

# cap.set(3,1280)
# cap.set(4,720)
# cap.set(10,70)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200)
cap.set(cv2.CAP_PROP_FPS, 10)


classNames = []
classFile = 'neuronet/coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'neuronet/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'neuronet/frozen_inference_graph.pb'




net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


class MOSSETracker:
    def __init__(self, frame, bbox):
        self.tracker = cv2.legacy.TrackerMOSSE_create()
        self.tracker.init(frame, bbox)
        self.object_id = 0
        self.object_ids = {}

    def track(self, frame):
        ret, bbox = self.tracker.update(frame)
        if ret:
            if self.object_id not in self.object_ids:
                self.object_ids[self.object_id] = bbox
                self.object_id += 1
        return ret, bbox
# object_tracker = CentroidTracker(max_disappeared_frames=10, distance_threshold=20)
# object_tracker = ObjectTracker()


ret, frame = cap.read()
if not ret:
    print("Error: Unable to read the video")
    exit()

frame = cv2.resize(frame, (320, 200), interpolation=cv2.INTER_AREA)
bbox = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
# Initialize MOSSE tracker
mosse_tracker = MOSSETracker(frame, bbox)


while True:
    success, frame = cap.read()
    # Resize the frame to 320x200 while maintaining aspect ratio
    frame = cv2.resize(frame, (320, 200), interpolation=cv2.INTER_AREA)

    classIds, confs, bbox = net.detect(frame, confThreshold=detection_threshold)
    print(f'{classIds=}, {bbox=}')
    ret, bbox = mosse_tracker.track(frame)
    if ret:
        # Tracking success
        x, y, w, h = [int(i) for i in bbox]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"ID: {mosse_tracker.object_id - 1}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 0), 2)
    else:
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    #
    # if len(classIds) != 0:
    #     index = 0
    #     for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
    #         if classNames[classId - 1] == 'person':
    #             cv2.rectangle(frame, box, color=(0, 255, 0), thickness=2)
    #             # cv2.putText(frame, classNames[classId - 1].upper(), (box[0] + 10, box[1] + 30),
    #             #             cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0), 2)
    #             # cv2.putText(frame, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
    #             #             cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0), 2)
    #             # cv2.putText(frame, "Id:" + str(object_id), (box[0] + 2, box[1] + 3),
    #             #             cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0), 2)

    cv2.imshow("Output", frame)
    if cv2.waitKey(1) == 27:  # Esc key
        break
