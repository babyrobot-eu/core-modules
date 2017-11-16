#!/usr/bin/env python

import rospy

from babyrobot.concept_net import client as cn_client
from babyrobot.concept_net import config as cn_config


def mock_embeddings():
    return [1, 2, 3], [1, 2, 3]


if __name__ == "__main__":
    rospy.init_node(cn_config.ROS_CONFIG.SEM_SIM_CLIENT_NODE)
    v1, v2 = mock_embeddings()
    sim = cn_client.get_semantic_similarity(v1, v2)
    rospy.logerr("Service responded with {}".format(sim))
