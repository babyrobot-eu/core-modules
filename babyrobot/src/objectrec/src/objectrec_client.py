#!/usr/bin/env python

import rospy

from babyrobot.objectrec.client import objectrec
from objectrec.utils import capture_frame, downscale_image, pad_image

if __name__ == "__main__":
    rospy.init_node('objectrec_client')

    # capture random image
    image = capture_frame()
    image = downscale_image(image)
    image = pad_image(image)
    recognized = objectrec(image)

    # with open('/tmp/image.pkl', 'rb') as f:
    #     image = cPickle.load(f)
    # recognized = objectrec(image)
    # json_recognized = json_message_converter.\
    #     convert_ros_message_to_json(recognized)
    # with open('/tmp/image.json', 'w') as f:
    #     json.dump(json_recognized, f)

    rospy.loginfo("Service responded with {}".format(recognized))
