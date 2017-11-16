#!/usr/bin/env python

import rospy

from babyrobot.concept_net import client as cn_client
from babyrobot.concept_net import config as cn_config
from rospy_message_converter import json_message_converter

import sys
import json


def mock_embeddings():
    return [1, 2, 3], [4, 5, 6], [7, 8, 9]


if __name__ == "__main__":
    rospy.init_node(cn_config.ROS_CONFIG.FUSION_CLIENT_NODE)
    text, visual, audio = sys.argv[1:]
    fused = cn_client.fuse_semantic_embeddings(text, visual, audio)
    json_recognized = json_message_converter. \
        convert_ros_message_to_json(fused)
    with open('/tmp/fused_embeddings.json', 'w') as f:
        json.dump(json_recognized, f)
    rospy.logerr("Service responded with {}".format(fused))
