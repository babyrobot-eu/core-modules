#!/usr/bin/env python

import rospy

from babyrobot.objectrec.client import objectrec
from babyrobot.objectrec.utils import capture_frame

if __name__ == "__main__":
    rospy.init_node('objectrec_client')

    # capture image
    image = capture_frame()

    recognized = objectrec(image)
    rospy.loginfo("Service responded with {}".format(recognized))
