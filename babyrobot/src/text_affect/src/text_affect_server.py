#!/usr/bin/env python
# -*- coding: utf-8 -*-
import uuid

import rospy
import nltk
import ujson as json
import math
import numpy as np

import babyrobot.lib.utils as br_utils

import babyrobot.text_affect.utils as ta_utils

from babyrobot_msgs.msg import TextAffectResult
from babyrobot_msgs.srv import TextAffect
from babyrobot_msgs.srv import TextAffectResponse

import babyrobot.text_affect.config as ta_config

LEXICON = ta_utils.load_lexicon(which='bing_liu')


def sigmoid(x):
    return 1 / (1 + math.exp(-x))


def get_valence(text):
    tokenized = nltk.word_tokenize(text.lower())
    total = []
    pos = []
    neg = []
    for word in tokenized:
        if word in LEXICON:
            entry = LEXICON[word]
            total.append(entry["polarity"])
    valence = np.tanh(sum(total)) + np.random.uniform(low=-0.1, high=0.1)
    if valence > 1:
        valence = 1
    if valence < -1:
        valence = -1
    return valence


def handle_text_affect(req):
    msg = TextAffectResult()
    msg.header.id = str(uuid.uuid1())
    msg.header.timestamp = rospy.Time.now()
    text = req.input.text
    msg.valence = get_valence(text)
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



#sentences = [
#     u"Ας παίξουμε ένα παιχνίδι.",
#     u"Είμαι μπερδεμένος.",
#     u"Καλημέρα σε όλους τους μικρούς μας φίλους!"
#    "Well done!.",
#    "Good morning everybody.",
#]
#for s in sentences:
#    print(s)
#    print(get_valence(br_utils.translate(s)))
