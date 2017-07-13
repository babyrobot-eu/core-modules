#!/usr/bin/env python
import rospy

from babyrobot_msgs.msg import ObjectRecognitionResult
from babyrobot_msgs.srv import ObjectRecognition, ObjectRecognitionResponse


def handle_objectrec(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))
    msg = ObjectRecognitionResult()
    msg.header.id = "Response sent from ObjectRec service"
    return ObjectRecognitionResponse(msg)


def objectrec_server():
    rospy.init_node('objectrec_server')
    rospy.Service('objectrec', ObjectRecognition, handle_objectrec)
    rospy.loginfo("Object Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    objectrec_server()
