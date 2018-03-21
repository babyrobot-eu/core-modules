#!/usr/bin/env python

import rospy

from babyrobot.text_affect import client as ta_client
from babyrobot.text_affect import config as ta_config
from rospy_message_converter import json_message_converter

import sys
import json

if __name__ == "__main__":
    rospy.init_node(ta_config.ROS_CONFIG.TEXT_AFFECT_CLIENT_NODE)
    with open(sys.argv[1]) as fd:
        text = ' '.join([l.strip() for l in fd.readlines()])
    affect_res = ta_client.get_text_affect_dims(text)
    rospy.logerr("Service responded with {}".format(affect_res))
    json_recognized = json_message_converter. \
        convert_ros_message_to_json(affect_res)
    with open('/tmp/text_affect_res.json', 'w') as f:
        json.dump(json_recognized, f)
    rospy.logerr("Service responded with {}".format(affect_res))
