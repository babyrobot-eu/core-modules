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
        self.processed_images = 0
        self.recorded = False
        self.sess = tf.Session(graph=detect_graph)
        self.recognized_image = ImageMsg()
        self.has_recognized = False
        self.bridge = CvBridge()
        self.boxes = None
        self.classes = None
        self.labels = []
        self.publish_labels = False
        self.scores = None
        self.pil_img = None
        rospy.init_node('iccs_objectrec', anonymous=True)
        self.pub = rospy.Publisher('/iccs/image/objectrec', ImageMsg, queue_size=100)
        self.pub_objects = rospy.Publisher('/iccs/objects', String, queue_size=100)
        rospy.Subscriber("/kinect1/qhd/image_color", ImageMsg, self.handle_image)
        self.pub_timer = rospy.Timer(rospy.Duration(1.0/20), self.pub_timer_callback)
        self.orec_timer = rospy.Timer(rospy.Duration(.5), self.orec_timer_callback)

    def orec_timer_callback(self, _):
        if self.processed_images >= 1:
            (self.boxes, self.classes, self.scores, num_detections) = orec_image(
                self.sess, detect_graph, self.pil_img)
            self.labels = [label_map[cls]['name'] for cls in self.classes]
            self.publish_labels = True
            # objects = list(zip(self.boxes, self.scores, self.classes))
            # debug_info_image(objects, label_map, CONFIG.model.threshold)
            self.processed_images = 0

    def pub_timer_callback(self, _):
        # rospy.loginfo('Update objectrec image')
        self.pub.publish(self.recognized_image)
        if self.publish_labels:
            self.pub_objects.publish(','.join(self.labels))
            self.publish_labels = False
            
    def cv2pil(self, img):
        return Image.fromarray(img)

    def debug_info_image(self, objects, category_index, threshold):
        """
        print debugging information, for the object recognition on an image
        :return:
        """
        for box, score, cls in objects:
            if score > threshold:
                rospy.loginfo("label:{}, score:{}, dist:{}".format(
                    category_index[cls]["name"],
                    score,
                    distance_box_to_center(box)))
        print("")

    def handle_image(self, image):
        # import pprint
        # pprint.pprint(label_map[51]['name'])
        self.processed_images += 1
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        pil_img = self.cv2pil(cv_image)
        pil_img = downscale_image(pil_img, baseWidth=480)
        pil_img = pad_image(pil_img)
        self.pil_img = pil_img
        # if self.processed_images == 10:
        #     (self.boxes, self.classes, self.scores, num_detections) = orec_image(
        #         self.sess, detect_graph, pil_img)
        #     # objects = list(zip(self.boxes, self.scores, self.classes))
        #     # debug_info_image(objects, label_map, CONFIG.model.threshold)
        #     self.processed_images = 0

        if self.boxes is None or self.scores is None or self.classes is None:
            self.recognized_image = image
            return
        #
        # # extract object image frames
        # frames, mask = extract_box_from_image(pil_img, objects,
        #                                       CONFIG.model.threshold)

        # print debugging information

        # Visualization of the results of a detection.
        img = get_rec_objects(pil_img,
                              self.boxes, self.classes, self.scores,
                              label_map,
                              CONFIG.model.threshold)

        # imsave("asdasdasd.png", img)
        # opencv_image = img
        # Convert RGB to BGR
        opencv_image = img[:, :, ::-1].copy()
        self.recognized_image = self.bridge.cv2_to_imgmsg(opencv_image, "rgb8")
        # self.recognized_image = self.bridge.cv2_to_imgmsg(img)

    def run(self):
        rospy.spin()
        self.pub_timer.shutdown()
        self.orec_timer.shutdown()
        # r = rospy.Rate(1)
        # while not rospy.is_shutdown():
        #     self.pub.publish(self.recognized_image)
        #     r.sleep()


if __name__ == '__main__':
    ObjectRec().run()
