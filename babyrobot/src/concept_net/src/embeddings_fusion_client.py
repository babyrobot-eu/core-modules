#!/usr/bin/env python

import rospy

from babyrobot.concept_net import client as cn_client
from babyrobot.concept_net import config as cn_config


def mock_embeddings():
    return [1, 2, 3], [4, 5, 6], [7, 8, 9]


if __name__ == "__main__":
    rospy.init_node(cn_config.ROS_CONFIG.FUSION_CLIENT_NODE)
    text, visual, audio = mock_embeddings()
    fused = cn_client.fuse_semantic_embeddings(text, visual, audio)
    rospy.logerr("Service responded with {}".format(fused))
