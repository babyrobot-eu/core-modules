#!/usr/bin/env python
import numpy as np
import rospy

from babyrobot.concept_net import config as cn_config
from babyrobot_msgs.msg import SemanticEmbedding
from babyrobot_msgs.srv import EmbeddingsFusion
from babyrobot_msgs.srv import EmbeddingsFusionResponse


def late_fusion(x, y, z, a=0.33, b=0.33):
    x, y, z = np.array(x), np.array(y), np.array(z)
    c = 1.0 - a - b
    return np.concatenate((a * x, b * y, c * z)).tolist()


def handle_fusion(req):
    resp = EmbeddingsFusionResponse()
    resp.fused = SemanticEmbedding()
    resp.fused.word = req.word
    resp.fused.found = 1
    text = req.modality_embeddings.text.embedding
    visual = req.modality_embeddings.visual.embedding
    audio = req.modality_embeddings.audio.embedding
    resp.fused.embedding = late_fusion(text, visual, audio)
    return resp


def embeddings_fusion_server():
    '''
    Initialize a ROS node and run the Embeddings Fusion service

    Args:

    Returns:
    '''
    rospy.init_node(cn_config.ROS_CONFIG.FUSION_SERVER_NODE)
    rospy.Service(cn_config.ROS_CONFIG.FUSION_SERVICE_NAME,
                  EmbeddingsFusion,
                  handle_fusion)
    rospy.loginfo("Embeddings fusion server started.")
    rospy.spin()


if __name__ == "__main__":
    embeddings_fusion_server()
