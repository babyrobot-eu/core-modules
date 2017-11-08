#!/usr/bin/env python

import rospy

from babyrobot.concept_net import client as cn_client
from babyrobot.concept_net import config as cn_config


if __name__ == "__main__":
    rospy.init_node(cn_config.ROS_CONFIG.SEM_SPACE_CLIENT_NODE)
    word = 'ball'
    semantic_embeddings = cn_client.get_semantic_embeddings(word)
    rospy.logerr("Service responded with {}".format(semantic_embeddings))
