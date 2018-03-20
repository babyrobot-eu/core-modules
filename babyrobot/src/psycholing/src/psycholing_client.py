#!/usr/bin/env python

import rospy

from babyrobot.psycholing import client as psy_client
from babyrobot.psycholing import config as psy_config
from rospy_message_converter import json_message_converter

import sys
import json

if __name__ == "__main__":
    rospy.init_node(psy_config.ROS_CONFIG.PSY_CLIENT_NODE)
    with open(sys.argv[1]) as fd:
        text = ' '.join([l.strip() for l in fd.readlines()])
    dimensions = psy_client.get_psycholing_dims(text)
    json_recognized = json_message_converter. \
        convert_ros_message_to_json(dimensions)
    with open('/tmp/psycholing_dimensions.json', 'w') as f:
        json.dump(json_recognized, f)
    rospy.logerr("Service responded with {}".format(dimensions))
