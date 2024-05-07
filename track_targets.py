import cv2
import numpy as np

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


class CentroidTracker:
    def __init__(self, max_disappeared_frames=50, distance_threshold=50):
        self.next_object_id = 0
        self.objects = {}
        self.disappeared = {}
        self.disappeared_frames = {}
        self.max_disappeared_frames = max_disappeared_frames
        self.distance_threshold = distance_threshold

    def register(self, centroid):
        """Registers a new object with its centroid."""
        self.objects[self.next_object_id] = centroid
        self.disappeared_frames[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        """Deregisters an object from tracking."""
        del self.objects[object_id]
        del self.disappeared_frames[object_id]

    def update(self, rects):
        """Updates the tracks of detected objects."""
        new_objects = {}
        for i, (x, y, w, h) in enumerate(rects):
            centroid = (int((x + x + w) / 2), int((y + y + h) / 2))
            object_id = -1
            for id, center in self.objects.items():
                # Convert tuples to NumPy arrays for element-wise subtraction
                center_array = np.array(center)
                centroid_array = np.array(centroid)
                distance = np.linalg.norm(center_array - centroid_array)
                if distance < self.distance_threshold:
                    object_id = id
                    self.disappeared[id] = 0
                    break
            if object_id < 0:
                self.register(centroid)
                object_id = self.next_object_id - 1

            new_objects[object_id] = centroid

        self.objects = new_objects
        print(f'{new_objects=}')
        return new_objects


net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

object_tracker = CentroidTracker(max_disappeared_frames=10, distance_threshold=20)

while True:
    success, frame = cap.read()
    # Resize the frame to 320x200 while maintaining aspect ratio
    frame = cv2.resize(frame, (320, 200), interpolation=cv2.INTER_AREA)

    classIds, confs, bbox = net.detect(frame, confThreshold=detection_threshold)
    print(f'{classIds=}, {bbox=}')
    object_ids = object_tracker.update(bbox)
    # print (f'{object_ids=}')

    if len(classIds) != 0:
        index = 0
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            if classNames[classId - 1] == 'person':
                cv2.rectangle(frame, box, color=(0, 255, 0), thickness=2)
                # cv2.putText(frame, classNames[classId - 1].upper(), (box[0] + 10, box[1] + 30),
                #             cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0), 2)
                # cv2.putText(frame, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                #             cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0), 2)
                # cv2.putText(frame, "Id:" + str(object_id), (box[0] + 2, box[1] + 3),
                #             cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0), 2)

    cv2.imshow("Output", frame)
    if cv2.waitKey(1) == 27:  # Esc key
        break
