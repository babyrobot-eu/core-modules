#!/usr/bin/env python
import base64
import math

import numpy as np
from std_msgs.msg import String

import pickle
import sys
import uuid

import rospy
import torch
from babyrobot.emorec_pytorch import config as emorec_pytorch_config
from babyrobot.speech_features import client as speech_feat_client
from babyrobot_msgs.msg import EmotionRecognitionResult
from torch.autograd import Variable
from speech_features import frame_breaker


sys.path.append(emorec_pytorch_config.Paths.src)


class SpeechAffect(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/speech_affect', EmotionRecognitionResult, queue_size=100)
        rospy.init_node('iccs_speech_affect', anonymous=True)
        rospy.Subscriber("/iccs/audio/recorder", String, self.handle_transcription)
        self.valence = 0
        self.text_affect_computed = False

    def extract_features(self, clip):
        """
        Feature extraction from an audio clip
        Args:
            clip ():

        Returns: A list of feature vectors

        """
        clip_array = np.frombuffer(clip, dtype=np.float64)
        segments = frame_breaker.get_frames(clip_array)
        segments_encoded = [base64.b64encode(s) for s in segments]
        segment_features = [
            speech_feat_client.extract_speech_features(
                s,
                opensmile_config=emorec_pytorch_config.ModelBaseline.opensmile_config,
                response_format='list'
            )
            for s in segments_encoded
        ]
        # extracted_feats = speech_feat_client.extract_speech_features(
        #     clip,
        #     opensmile_config=emorec_pytorch_config.ModelBaseline.opensmile_config,
        #     response_format='list'
        # )
        # feats = np.array([f.feature_value for f in extracted_feats])
        return segment_features

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

    def handle_audio(self, audio):
        self.valence = self.get_valence(audio.data)
        self.text_affect_computed = True
        rospy.loginfo("Text valence: {}".format(self.valence))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.text_affect_computed:
                self.pub.publish(self.valence)
                self.text_affect_computed = False
                r.sleep()


if __name__ == '__main__':
    SpeechAffect().run()
