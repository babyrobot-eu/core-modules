"""
Config file for object recognition

Tensorflow detection model zoo:
https://github.com/tensorflow/models/blob/master/object_detection/g3doc/
"""

PROJECT_ROOT = '/home/christos/PycharmProjects/babyrobot-integration/'


class MODEL_PATHS:
    models = PROJECT_ROOT + 'babyrobot/objectrec/models'
    images = PROJECT_ROOT + 'babyrobot/objectrec/images/lab'


class MODEL:
    name = 'ssd_mobilenet_v1_coco_11_06_2017'
    labels = 'mscoco_label_map.pbtxt'
    threshold = 0.35
    classes = 90
    repo = 'http://download.tensorflow.org/models/object_detection/'
    visualize = True
    paths = MODEL_PATHS


class OBJECTREC:
    debug = True
    model = MODEL
