#!/usr/bin/env python

import rospy

from babyrobot.concept_net import client as cn_client
from babyrobot.concept_net import config as cn_config
from rospy_message_converter import json_message_converter

import sys
import json


def mock_embeddings():
    return [1, 2, 3], [1, 2, 3]


if __name__ == "__main__":
    rospy.init_node(cn_config.ROS_CONFIG.SEM_SIM_CLIENT_NODE)
    v1, v2 = sys.argv[1:]
    sim = cn_client.get_semantic_similarity(v1, v2)
    json_recognized = json_message_converter. \
        convert_ros_message_to_json(sim)
    with open('/tmp/semantic_similarity.json', 'w') as f:
        json.dump(json_recognized, f)
    rospy.logerr("Service responded with {}".format(sim))
