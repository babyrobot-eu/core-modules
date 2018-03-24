#!/usr/bin/env python
import rospy
import uuid
import nltk
from std_msgs.msg import String
import babyrobot.psycholing.utils as psy_utils

from babyrobot_msgs.msg import PsycholingResult
from babyrobot_msgs.msg import PsycholingDim


class Psycholing(object):
    def __init__(self):
        self.valid_dimensions = ['affect', 'percept', 'cogproc', 'drives', 'social', 'anx']

        self.lexicon = psy_utils.load_lexicon()

        self.dimension_map = psy_utils.load_dimensions()

        self.valid_dimension_map = {
            k: v for k, v in self.dimension_map.items() if k in self.valid_dimensions
        }

        self.pub = rospy.Publisher('/iccs/cognistates', PsycholingResult, queue_size=100)
        rospy.init_node('iccs_cognistates', anonymous=True)
        rospy.Subscriber("/iccs/translate", String, self.handle_transcription)
        self.cogni = PsycholingResult()
        self.cogni_computed = False

    def get_psycholing_dims(self, text):
        psy_dims = {
            k: 0 for k in self.valid_dimensions
        }
        tokenized = nltk.word_tokenize(text.lower())
        for word in tokenized:
            if word in self.lexicon:
                for k in psy_dims.keys():
                    dim = self.valid_dimension_map[k]
                    count = self.lexicon[word][dim]
                    psy_dims[k] += count
        for k in psy_dims.keys():
            psy_dims[k] = float(psy_dims[k]) / len(tokenized)
        return psy_dims

    def handle_transcription(self, transcription):
        psy_dims = self.get_psycholing_dims(transcription.data)
        self.cogni.header.id = str(uuid.uuid1())
        self.cogni.header.timestamp = rospy.Time.now()
        for k, v in psy_dims.items():
            psy = PsycholingDim()
            psy.dimension = k
            psy.count = v
            self.cogni.dimensions.append(psy)
        self.cogni.input = transcription
        self.cogni_computed = True
        rospy.loginfo("Cognitive states: affect={}".format(psy_dims['affect']))
        rospy.loginfo("Cognitive states: percept={}".format(psy_dims['percept']))
        rospy.loginfo("Cognitive states: cognitive load={}".format(psy_dims['cogproc']))
        rospy.loginfo("Cognitive states: drives={}".format(psy_dims['drives']))
        rospy.loginfo("Cognitive states: social={}".format(psy_dims['social']))
        rospy.loginfo("Cognitive states: stress={}".format(psy_dims['anx']))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cogni_computed:
                self.pub.publish(self.cogni)
                self.cogni_computed = False
                r.sleep()


if __name__ == '__main__':
    Psycholing().run()
