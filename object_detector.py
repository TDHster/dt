import cv2

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
