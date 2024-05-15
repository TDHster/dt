import cv2


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

    def detect_objects(self, frame, confThreshold=0.3):
        classIds, confs, bbox = self.net.detect(frame, confThreshold=confThreshold)
        return classIds, confs, bbox

    @property
    def object_class_names(self):
        return self._object_class_name


def filter_by_target_class_id(classIds, bbox, names_list, target_class_name='person'):
    if len(classIds):
        # Efficient filtering using boolean indexing
        # keep_indices = classIds == object_class_names.index(target_class_name) + 1  # Indices where specified class ID (1 is person)
        keep_indices = classIds == names_list.index(target_class_name) + 1  # Indices where specified class ID (1 is person)
        # keep_indices = classIds == 1  # Indices where specified class ID (1 is person)
        classIds = classIds[keep_indices]
        bbox = bbox[keep_indices]
        return classIds, bbox

