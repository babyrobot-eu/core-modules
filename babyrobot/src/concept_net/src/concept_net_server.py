#!/usr/bin/env python
import rospy
import ujson as json

from babyrobot.concept_net import config as cn_config
from babyrobot_msgs.msg import SemanticEmbedding
from babyrobot_msgs.srv import ConceptNet
from babyrobot_msgs.srv import ConceptNetResponse

# Semantic spaces are stored in memory in the format
# SPACE = {'dim': vector dimensionality,
#          'default_not_found': default value for non existing words in space,
#          'space': dict of 'word': 'embedding' key value pairs}
TEXT_SEMANTIC_SPACE = {}
VISUAL_SEMANTIC_SPACE = {}
AUDIO_SEMANTIC_SPACE = {}


def get_word_embedding_in_space(space, word):
    # TODO: Change this to boolean type.
    # Currently the ROS wiki is down so using int
    embedding, found = space['default_not_found'], 0
    if word in space['space']:
        embedding, found = space['space']['word'], 1
    return embedding, found


def load_semantic_space(semantic_space_file):
    space = {}
    with open(semantic_space_file, 'r') as fd:
        space = json.load(fd)

    dim = len(space.itervalues().next())
    res = {
        'space': space,
        'dim': dim,
        'default_not_found': [0 for _ in range(dim)]
    }
    return res


def handle_concept_net(req):
    resp = ConceptNetResponse()
    resp.text = SemanticEmbedding()
    resp.visual = SemanticEmbedding()
    resp.audio = SemanticEmbedding()
    resp.text.word = req.word
    resp.text.embedding, resp.text.found = get_word_embedding_in_space(
        TEXT_SEMANTIC_SPACE, req.word)
    resp.visual.word = req.word
    resp.visual.embedding, resp.visual.found = get_word_embedding_in_space(
        VISUAL_SEMANTIC_SPACE, req.word)
    resp.audio.word = req.word
    resp.audio.embedding, resp.audio.found = get_word_embedding_in_space(
        AUDIO_SEMANTIC_SPACE, req.word)
    return resp


def concept_net_server():
    global TEXT_SEMANTIC_SPACE, VISUAL_SEMANTIC_SPACE, AUDIO_SEMANTIC_SPACE
    '''
    Initialize a ROS node and run the ConceptNet service

    Args:

    Returns:
    '''
    rospy.init_node(cn_config.ROS_CONFIG.SERVER_NODE)
    rospy.Service(cn_config.ROS_CONFIG.SERVICE_NAME,
                  ConceptNet,
                  handle_concept_net)
    TEXT_SEMANTIC_SPACE = load_semantic_space(cn_config.SEMANTIC_SPACE.TEXT)
    VISUAL_SEMANTIC_SPACE = load_semantic_space(
        cn_config.SEMANTIC_SPACE.VISUAL)
    AUDIO_SEMANTIC_SPACE = load_semantic_space(cn_config.SEMANTIC_SPACE.AUDIO)
    rospy.loginfo("Conceptual Networks server started.")
    rospy.spin()


if __name__ == "__main__":
    concept_net_server()
