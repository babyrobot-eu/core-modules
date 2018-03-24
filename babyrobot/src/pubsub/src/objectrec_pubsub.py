#!/usr/bin/env python
import base64
from io import BytesIO
import numpy as np
import rospy
from objectrec.config import OBJECTREC as CONFIG
from objectrec.utils import check_model, load_frozen_model, get_labels_map, downscale_image, pad_image, orec_image, \
    extract_box_from_image, debug_info_image, get_rec_objects, distance_box_to_center
from std_msgs.msg import String
from PIL import Image
import cv2
from sensor_msgs.msg import Image as ImageMsg
import wave
import sox
import contextlib
from babyrobot.lib import config as br_config
from scipy.io.wavfile import read as wavfile_read

import os
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from matplotlib.image import imsave

# Download Model
check_model(CONFIG.model.name)

# Load a (frozen) Tensorflow model into memory.
detect_graph = load_frozen_model(CONFIG.model.name)

# Loading label map
label_map = get_labels_map(CONFIG.model.labels,
                           CONFIG.model.classes)


class ObjectRec(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/image/objectrec', ImageMsg, queue_size=1)
        self.recorded = False
        rospy.init_node('iccs_objectrec', anonymous=True)
        rospy.Subscriber("/kinect1/qhd/image_color", ImageMsg, self.handle_image)
        self.sess = tf.Session(graph=detect_graph)
        self.recognized_image = None
        self.has_recognized = False
        self.bridge = CvBridge()

    def cv2pil(self, img):
        return Image.fromarray(img)

    def debug_info_image(self, objects, category_index, threshold):
        """
        print debugging information, for the object recognition on an image
        :return:
        """
        for box, score, cls in objects:
            if score > threshold:
                print("label:{}, score:{}, dist:{}".format(
                    category_index[cls]["name"],
                    score,
                    distance_box_to_center(box)))
        print("")

    def handle_image(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        pil_img = self.cv2pil(cv_image)
        pil_img = downscale_image(pil_img, baseWidth=640)
        pil_img = pad_image(pil_img)

        # pil_img.show()

        (boxes, classes, scores, num_detections) = orec_image(self.sess,
                                                              detect_graph,
                                                              pil_img)

        objects = list(zip(boxes, scores, classes))
        #
        # # extract object image frames
        # frames, mask = extract_box_from_image(pil_img, objects,
        #                                       CONFIG.model.threshold)

        # print debugging information
        debug_info_image(objects, label_map, CONFIG.model.threshold)

        # Visualization of the results of a detection.
        img = get_rec_objects(pil_img,
                              boxes, classes, scores,
                              label_map,
                              CONFIG.model.threshold)

        # imsave("asdasdasd.png", img)
        # opencv_image = img
        # Convert RGB to BGR
        # opencv_image = opencv_image[:, :, ::-1].copy()
        self.recognized_image = self.bridge.cv2_to_imgmsg(img, "rgb8")
        # self.recognized_image = self.bridge.cv2_to_imgmsg(img)
        self.has_recognized = True
        rospy.loginfo('Object recognition has run')

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.has_recognized:
                self.pub.publish(self.recognized_image)
                r.sleep()


if __name__ == '__main__':
    ObjectRec().run()
