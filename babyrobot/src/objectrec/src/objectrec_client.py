#!/usr/bin/env python

import rospy

from babyrobot.objectrec.client import objectrec
from rospy_message_converter import json_message_converter

import cPickle
import json

if __name__ == "__main__":
    rospy.init_node('objectrec_client')

    # capture random image
    # image = capture_frame()

    with open('/tmp/image.pkl', 'rb') as f:
        image = cPickle.load(f)
    recognized = objectrec(image)
    json_recognized = json_message_converter.\
        convert_ros_message_to_json(recognized)
    with open('/tmp/image.json', 'w') as f:
        json.dump(json_recognized, f)

    rospy.loginfo("Service responded with {}".format(recognized))
