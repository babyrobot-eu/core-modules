#!/usr/bin/env python
import uuid

import rospy
import ujson as json

import babyrobot.text_affect.utils as ta_utils

from babyrobot_msgs.msg import TextAffectResult
from babyrobot_msgs.srv import TextAffect
from babyrobot_msgs.srv import TextAffectResponse

import babyrobot.text_affect.config as ta_config

LEXICON = ta_utils.load_lexicon()


def handle_text_affect(req):
    msg = TextAffectResult()
    msg.header.id = str(uuid.uuid1())
    msg.header.timestamp = rospy.Time.now()
    text = req.input.text
    # TODO: Add valence calculation
    msg.valence = 0
    msg.input = req.input.text
    return TextAffectResponse(msg)


def text_affect_server():
    '''
    Initialize a ROS node and run the TextAffect service

    Args:

    Returns:
    '''
    rospy.init_node(ta_config.ROS_CONFIG.TEXT_AFFECT_SERVER_NODE)
    rospy.Service(ta_config.ROS_CONFIG.TEXT_AFFECT_SERVICE_NAME,
                  TextAffect,
                  handle_text_affect)
    rospy.loginfo("TextAffect server started.")
    rospy.spin()


if __name__ == "__main__":
    text_affect_server()
