#!/usr/bin/env python
import math
import operator

import rospy

from babyrobot.concept_net import config as cn_config
from babyrobot_msgs.srv import SemanticSimilarity
from babyrobot_msgs.srv import SemanticSimilarityResponse


def cos_similarity(v1, v2):
    if len(v1) != len(v2):
        return -1

    def dot_product(v1, v2):
        return sum(map(operator.mul, v1, v2))

    prod = dot_product(v1, v2)
    len1 = math.sqrt(dot_product(v1, v1))
    len2 = math.sqrt(dot_product(v2, v2))
    return float(prod) / (len1 * len2)


def handle_semantic_similarity(req):
    resp = SemanticSimilarityResponse()
    resp.similarity_score = cos_similarity(
        req.embeddings.v1, req.embeddings.v2)
    return resp


def semantic_similarity_server():
    '''
    Initialize a ROS node and run the Semantic Similarity service

    Args:

    Returns:
    '''
    rospy.init_node(cn_config.ROS_CONFIG.SEM_SIM_SERVER_NODE)
    rospy.Service(cn_config.ROS_CONFIG.SEM_SIM_SERVICE_NAME,
                  SemanticSimilarity,
                  handle_semantic_similarity)
    rospy.loginfo("Semantic similarity server started.")
    rospy.spin()


if __name__ == "__main__":
    semantic_similarity_server()
