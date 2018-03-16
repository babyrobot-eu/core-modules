"""
Config file for object recognition

Tensorflow detection model zoo:
https://github.com/tensorflow/models/blob/master/object_detection/g3doc/
"""

import os

import babyrobot.lib.config as br_config


class MODEL_PATHS(object):
    models = os.path.join(br_config.BASE_PATH, 'babyrobot/objectrec/models')
    images = os.path.join(br_config.BASE_PATH,
                          'babyrobot/objectrec/images/rest')


class MODEL(object):
    # name = 'ssd_inception_v2_coco_11_06_2017'
    name = 'faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017'
    labels = 'mscoco_label_map.pbtxt'
    # name = 'faster_rcnn_inception_resnet_v2_atrous_oid_2018_01_28'
    # labels = 'oid_bbox_trainable_label_map.pbtxt'
    threshold = 0.3
    classes = 90
    repo = 'http://download.tensorflow.org/models/object_detection/'
    paths = MODEL_PATHS


class OBJECTREC(object):
    debug = True
    model = MODEL
