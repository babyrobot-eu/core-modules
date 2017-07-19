#!/usr/bin/env python

import numpy as np
import rospy
from babyrobot_msgs.msg import Frame
from babyrobot_msgs.srv import ObjectRecognition

from tf.utils import capture_frame


def objectrec(metadata):
    rospy.wait_for_service('objectrec')
    try:
        objectrec = rospy.ServiceProxy('objectrec', ObjectRecognition)

        frame = Frame()

        # capture image
        image = capture_frame()

        # show for debugging purposes
        # image.show()

        width, height = image.size

        # set parameters of Frame object
        frame.width = width
        frame.height = height
        frame.encoding = image.format

        # todo: find a more elegant way to serialize the image data
        frame.data = np.array(image.getdata(),
                              dtype=np.uint8).flatten().tolist()

        # make the request to the object recognition server
        objectrec_response = objectrec(frame, metadata)

        return objectrec_response.recognized
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node('objectrec_client')
    metadata = "Hello from objectrec client"
    recognized = objectrec(metadata)
    rospy.logerr("Service responded with {}".format(recognized.header.id))
