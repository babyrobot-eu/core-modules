#!/usr/bin/env python

import rospy

from babyrobot_msgs.msg import Frame
from babyrobot_msgs.srv import ObjectRecognition


def objectrec(metadata):
    rospy.wait_for_service('objectrec')
    try:
        objectrec = rospy.ServiceProxy('objectrec', ObjectRecognition)
        frame = Frame()
        objectrec_response = objectrec(frame, metadata)
        return objectrec_response.recognized
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node('objectrec_client')
    metadata = "Hello from objectrec client"
    recognized = objectrec(metadata)
    rospy.logerr("Service responded with {}".format(recognized.header.id))
