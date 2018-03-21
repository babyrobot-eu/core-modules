#!/usr/bin/env python
import uuid

import rospy
import ujson as json

import babyrobot.psycholing.utils as psy_utils

from babyrobot_msgs.msg import PsycholingDim
from babyrobot_msgs.msg import PsycholingResult
from babyrobot_msgs.srv import Psycholing
from babyrobot_msgs.srv import PsycholingResponse

import babyrobot.psycholing.config as psy_config

VALID_DIMENSIONS = ['affect', 'percept', 'cogproc', 'drives', 'social']

LEXICON = psy_utils.load_lexicon()

DIMENSION_MAP = psy_utils.load_dimensions()

VALID_DIMENSION_MAP = {
    k: v for k, v in DIMENSION_MAP.items() if k in VALID_DIMENSIONS
}


def get_psycholing_dims(text):
    psy_dims = {
        k: 0 for k in VALID_DIMENSIONS
    }
    for word in text.lower().split():
        if word in LEXICON:
            print word + " in lexicon"
            for k in psy_dims.keys():
                print k
                dim = VALID_DIMENSION_MAP[k]
                count = LEXICON[word][dim]
                psy_dims[k] += count
    return psy_dims


def handle_psycholing(req):
    msg = PsycholingResult()
    msg.header.id = str(uuid.uuid1())
    msg.header.timestamp = rospy.Time.now()
    psy_dims = get_psycholing_dims(req.input.text)
    for k, v in psy_dims.items():
        psy = PsycholingDim()
        psy.dimension = k
        psy.count = v
        msg.dimensions.append(psy)
    msg.input = req.input.text
    return PsycholingResponse(msg)


def psycholing_server():
    '''
    Initialize a ROS node and run the Psycholing service

    Args:

    Returns:
    '''
    rospy.init_node(psy_config.ROS_CONFIG.PSY_SERVER_NODE)
    rospy.Service(psy_config.ROS_CONFIG.PSY_SERVICE_NAME,
                  Psycholing,
                  handle_psycholing)
    rospy.loginfo("Psycholing server started.")
    rospy.spin()


if __name__ == "__main__":
    psycholing_server()
