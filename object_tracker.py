import cv2
import numpy as np


class CentroidTracker:
    def __init__(self, max_disappeared_frames=5, distance_threshold=5):
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
            centroid = (int((x + x + w) / 2), int((y + y + h) / 2), w, h)
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
        # print(f'new_objects={new_objects}')
        return new_objects


class MOSSETracker:
    def __init__(self, adaptive=False):
        self.adaptive = adaptive
        self.mosse = cv2.legacy.TrackerMOSSE_create()
        self.next_object_id = 0
        self.objects = {}

    def init(self, frame, bbox):
        """Initializes the tracker with the first frame and bounding box."""
        object_id = self.next_object_id
        ok = self.mosse.init(frame, bbox)
        self.objects[object_id] = bbox
        self.next_object_id += 1
        return ok, object_id

    def update(self, bboxes):
        """Updates the tracker's state with the current frame and array of bounding boxes."""
        object_ids = list(self.objects.keys())
        for object_id in object_ids:
            bbox = self.objects[object_id]
            ok, new_bbox = self.mosse.update(bboxes)
            if ok:
                self.objects[object_id] = new_bbox
            else:
                del self.objects[object_id]
        return None  #TODO need to return object id and centroid coords

    def get_object_id(self, bbox):
        """Returns the object ID associated with the given bounding box."""
        for object_id, bbox_ref in self.objects.items():
            if np.array_equal(bbox, bbox_ref):
                return object_id
        return -1
