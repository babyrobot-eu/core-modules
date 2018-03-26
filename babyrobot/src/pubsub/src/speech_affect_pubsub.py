#!/usr/bin/env python
import base64
import math

import io
import numpy as np
from std_msgs.msg import String

from scipy.io.wavfile import read as wav_read
from scipy.io.wavfile import write as wav_write
import pickle
import sys
import uuid

import rospy
import torch
from babyrobot.emorec_pytorch import config as emorec_pytorch_config
from babyrobot.speech_features import client as speech_feat_client
from babyrobot_msgs.msg import SpeechAffectResult
from torch.autograd import Variable
from speech_features import frame_breaker
from babyrobot_msgs.msg import Ints
from rospy.numpy_msg import numpy_msg


sys.path.append(emorec_pytorch_config.Paths.src)


class SpeechAffect(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/speech_affect', SpeechAffectResult, queue_size=100)
        rospy.init_node('iccs_speech_affect', anonymous=True)
        rospy.Subscriber("/iccs/audio/recorder_numpy", numpy_msg(Ints), self.handle_audio)
        self.emorec_result = SpeechAffectResult()
        self.speech_affect_computed = False

        self.model_path = emorec_pytorch_config.Paths.checkpoint
        self.data_manager_path = emorec_pytorch_config.Paths.data_manager
        self._model = torch.load(self.model_path)
        self._data_manager = pickle.load(open(self.data_manager_path, 'rb'))
        self._model.eval()
        print("Speech affect ready")

    def extract_feats_for_segment(self, s):
        return speech_feat_client.extract_speech_features(
            s,
            opensmile_config=emorec_pytorch_config.ModelBaseline.opensmile_config,
            response_format='list'
        )

    def np2base64(self, s, sr):
        wav_write('/tmp/tmp.wav', sr, s)
        with open('/tmp/tmp.wav', 'r') as fd:
            s2 = fd.read()
        return s2

    def extract_features(self, clip):
        """
        Feature extraction from an audio clip
        Args:
            clip ():

        Returns: A list of feature vectors

        """
        #sr, clip_array = wav_read(io.BytesIO(clip.data))
        sr = 16000
        # clip_decoded = base64.decodestring(clip.data)
        # clip_array = np.frombuffer(clip_decoded, dtype=np.float16)
        print(clip.data)
        clip_array = np.array(clip.data)
        if clip_array.ndim > 1:
            clip_array = clip_array[:, 0]
        segments = frame_breaker.get_frames(clip_array, sample_rate=sr)
        segments_encoded = [self.np2base64(s, sr) for s in segments]
        segment_features = [
            [f.feature_value for f in self.extract_feats_for_segment(s).features]
            for s in segments_encoded
        ]
        return segment_features

    def handle_audio(self, audio):
        self.emorec_result = SpeechAffectResult()
        self.emorec_result.header.frame_id = str(uuid.uuid1())
        # self.emorec_result.related_segment_id = self.emorec_result.header.id
        self.emorec_result.header.stamp = rospy.Time.now()
        clip = self.extract_features(audio)
        input, length = self._data_manager.prep_sample(clip)
        input = input.astype('float32')

        # convert the input to torch tensors
        value = torch.from_numpy(input)
        length = torch.from_numpy(np.array([length]))

        test_value = Variable(value, volatile=True)
        test_length = Variable(length, volatile=True)

        # make it float and insert a fake batch dimension
        test_value = test_value.float()
        test_value = test_value.unsqueeze(0)

        # cast the length tensor tos a LongTensor
        test_length = test_length.long()

        # pass the sample and it's length to the model and get the predictions
        cat_outputs, cont_outputs = self._model(test_value, test_length)

        # move the output tensors to the cpu and convert them to numpy arrays
        cat_outputs = cat_outputs.data.cpu().numpy()
        intensities = cont_outputs.data.cpu().numpy()

        # get the predicted polarity
        print(cat_outputs)
        pred_cat = np.argmax(cat_outputs[::-1])
        polarity = self._data_manager.label_cat_encoder.inverse_transform(pred_cat)

        self.emorec_result.polarity = polarity
        self.emorec_result.intensities = intensities

        self.speech_affect_computed = True
        rospy.loginfo("Speech emotion polarity: {}".format(polarity))
        rospy.loginfo("Speech emotion intensities: {}".format(intensities))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.speech_affect_computed:
                self.pub.publish(self.emorec_result)
                self.speech_affect_computed = False
                self.emorec_result = EmotionRecognitionResult()
                r.sleep()


if __name__ == '__main__':
    SpeechAffect().run()
