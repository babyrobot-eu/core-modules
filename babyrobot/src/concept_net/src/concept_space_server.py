#!/usr/bin/env python
import uuid

import rospy
import ujson as json

from babyrobot.concept_net import config as cn_config
from babyrobot_msgs.msg import SemanticEmbedding
from babyrobot_msgs.msg import SemanticSpaceResult
from babyrobot_msgs.srv import SemanticSpace
from babyrobot_msgs.srv import SemanticSpaceResponse

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


def handle_semantic_space(req):
    msg = SemanticSpaceResult()
    msg.header.id = str(uuid.uuid1())
    msg.header.timestamp = rospy.Time.now()
    msg.text = SemanticEmbedding()
    msg.visual = SemanticEmbedding()
    msg.audio = SemanticEmbedding()
    msg.text.word = req.query.word
    msg.text.embedding, msg.text.found = get_word_embedding_in_space(
        TEXT_SEMANTIC_SPACE, req.query.word)
    msg.visual.word = req.query.word
    msg.visual.embedding, msg.visual.found = get_word_embedding_in_space(
        VISUAL_SEMANTIC_SPACE, req.query.word)
    msg.audio.word = req.query.word
    msg.audio.embedding, msg.audio.found = get_word_embedding_in_space(
        AUDIO_SEMANTIC_SPACE, req.query.word)
    return SemanticSpaceResponse(msg)


def semantic_space_server():
    global TEXT_SEMANTIC_SPACE, VISUAL_SEMANTIC_SPACE, AUDIO_SEMANTIC_SPACE
    '''
    Initialize a ROS node and run the Semantic Space service

    Args:

    Returns:
    '''
    rospy.init_node(cn_config.ROS_CONFIG.SEM_SPACE_SERVER_NODE)
    rospy.Service(cn_config.ROS_CONFIG.SEM_SPACE_SERVICE_NAME,
                  SemanticSpace,
                  handle_semantic_space)
    TEXT_SEMANTIC_SPACE = load_semantic_space(cn_config.SEMANTIC_SPACE.TEXT)
    VISUAL_SEMANTIC_SPACE = load_semantic_space(
        cn_config.SEMANTIC_SPACE.VISUAL)
    AUDIO_SEMANTIC_SPACE = load_semantic_space(cn_config.SEMANTIC_SPACE.AUDIO)
    rospy.loginfo("Semantic Space server started.")
    rospy.spin()


if __name__ == "__main__":
    semantic_space_server()
