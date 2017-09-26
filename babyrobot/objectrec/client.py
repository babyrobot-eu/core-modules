#!/usr/bin/env python

import numpy as np
import rospy
from babyrobot_msgs.msg import Frame
from babyrobot_msgs.srv import ObjectRecognition

from babyrobot.objectrec.utils import (visualize_objectrec_response,
                                       get_labels_map)
from babyrobot.objectrec.config import OBJECTREC as CONFIG

# Loading label map. Load it once to speed things up
LABEL_MAP = get_labels_map(CONFIG.model.labels, CONFIG.model.classes)


def objectrec(image):
    rospy.wait_for_service('objectrec')
    try:
        objectrec_srv = rospy.ServiceProxy('objectrec', ObjectRecognition)
        frame = Frame()

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
        metadata = ''
        objectrec_response = objectrec_srv(frame, metadata)

        if CONFIG.debug:
            visualize_objectrec_response(image,
                                         objectrec_response,
                                         LABEL_MAP,
                                         CONFIG.model.threshold)

        return objectrec_response.recognized
    except rospy.ServiceException as ex:
        rospy.logerr("Service call failed: {}".format(ex))
