#!/usr/bin/env python
import math

import numpy as np
import rospy
import nltk
from babyrobot_msgs.msg import TimedString, TimedFloat64
from std_msgs.msg import Float64, String
import babyrobot.text_affect.utils as ta_utils


class TextAffect(object):
    def __init__(self):

        self.lexicon = ta_utils.load_lexicon(which='bing_liu')

        self.pub = rospy.Publisher('/iccs/text_affect', TimedFloat64, queue_size=100)
        rospy.init_node('iccs_text_affect', anonymous=True)
        rospy.Subscriber("/iccs/translate", TimedString, self.handle_transcription)
        self.valence = TimedFloat64()
        self.text_affect_computed = False

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

    def get_valence(self, text):
        tokenized = nltk.word_tokenize(text.lower())
        total = []
        pos = []
        neg = []
        for word in tokenized:
            if word in self.lexicon:
                entry = self.lexicon[word]
                total.append(entry["polarity"])
        valence = np.tanh(sum(total)) + np.random.uniform(low=-0.1, high=0.1)
        if valence > 1:
            valence = 1
        if valence < -1:
            valence = -1
        return valence

    def handle_transcription(self, transcription):
        self.valence.data = self.get_valence(transcription.data)
        self.valence.header.stamp = rospy.Time().now()
        self.text_affect_computed = True
        rospy.loginfo("Text valence: {}".format(self.valence.data))

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.text_affect_computed:
                self.pub.publish(self.valence)
                self.text_affect_computed = False
            r.sleep()


if __name__ == '__main__':
    TextAffect().run()
