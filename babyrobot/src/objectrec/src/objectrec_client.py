#!/usr/bin/env python

import numpy as np
import rospy
from babyrobot_msgs.msg import Frame
from babyrobot_msgs.srv import ObjectRecognition

from tf.config import OBJECTREC as CONFIG
from tf.utils import capture_frame, visualize_objectrec_response, \
    get_labels_map


def objectrec(metadata):
    rospy.wait_for_service('objectrec')
    try:
        objectrec = rospy.ServiceProxy('objectrec', ObjectRecognition)
        frame = Frame()

        # capture image
        image = capture_frame()

        if CONFIG.debug:
            image.show()

        width, height = image.size

        # set parameters of Frame object
        frame.width = width
        frame.height = height
        frame.encoding = image.format

        # todo: find a more elegant way to serialize the image data
        frame.data = np.array(image.getdata(),
                              dtype=np.uint8).flatten().tolist()

        # make request to the object recognition server
        objectrec_response = objectrec(frame, metadata)

        if CONFIG.debug:
            visualize_objectrec_response(image,
                                         objectrec_response,
                                         label_map,
                                         CONFIG.model.threshold)

        return objectrec_response.recognized
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))


if __name__ == "__main__":
    # Loading label map
    label_map = get_labels_map(CONFIG.model.labels,
                               CONFIG.model.classes)

    rospy.init_node('objectrec_client')
    metadata = "Hello from objectrec client"
    recognized = objectrec(metadata)
    rospy.logerr("Service responded with {}".format(recognized.header.id))
