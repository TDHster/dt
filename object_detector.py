import cv2
import numpy as np

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

    def _filter(self, class_ids, bbox, target_class_names=('person', 'car')):
        """Filters detections based on a list of target class names.

        Args:
            class_ids (List[int]): A list of class IDs for detected objects.
            bbox (List[List[int]]): A list of bounding boxes for detected objects,
                                     where each inner list represents a box (x_min, y_min, x_max, y_max).
            target_class_names (List[str], optional): A list of target class names
                                                       to filter detections by. Defaults to ['person'].

        Returns:
            tuple: A tuple containing two filtered lists:
                   - filtered_class_ids (List[int]): A list of class IDs for detected objects
                                                    that belong to the target classes.
                   - filtered_bbox (List[List[int]]): A list of bounding boxes for the
                                                      filtered detections.
        """
        if not len(class_ids) or not len(target_class_names):
            return (), ()  # Return empty lists if no class IDs or target classes

        # Get class ID indices corresponding to target class names (efficiently)
        class_id_to_index = {name: i for i, name in enumerate(self.object_class_names)}
        target_class_indices = [class_id_to_index[name] for name in target_class_names if name in class_id_to_index]

        # Efficient boolean indexing for filtering (vectorized approach)
        keep_indices = np.isin(class_ids, target_class_indices)
        filtered_class_ids = class_ids[keep_indices]
        filtered_bbox = bbox[keep_indices]

        return filtered_class_ids, filtered_bbox