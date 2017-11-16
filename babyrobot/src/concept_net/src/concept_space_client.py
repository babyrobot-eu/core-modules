#!/usr/bin/env python

import rospy

from babyrobot.concept_net import client as cn_client
from babyrobot.concept_net import config as cn_config
from rospy_message_converter import json_message_converter

import sys
import json

if __name__ == "__main__":
    rospy.init_node(cn_config.ROS_CONFIG.SEM_SPACE_CLIENT_NODE)
    word = sys.argv[1]
    semantic_embeddings = cn_client.get_semantic_embeddings(word)
    json_recognized = json_message_converter. \
        convert_ros_message_to_json(semantic_embeddings)
    with open('/tmp/semantic_embeddings.json', 'w') as f:
        json.dump(json_recognized, f)
    rospy.logerr("Service responded with {}".format(semantic_embeddings))
