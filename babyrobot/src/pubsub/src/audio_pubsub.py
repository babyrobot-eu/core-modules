#!/usr/bin/env python
import base64

import numpy as np
import rospy
from std_msgs.msg import String
import wave
import sox
import contextlib
from babyrobot.lib import config as br_config
from scipy.io.wavfile import read as wavfile_read
from babyrobot_msgs.msg import Ints

from rospy.numpy_msg import numpy_msg


class AudioRecorder(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/audio/recorder', String, queue_size=100)
        self.pub_np = rospy.Publisher('/iccs/audio/recorder_numpy', numpy_msg(Ints), queue_size=100)
        self.utterance = ''
        self.utterance_count = 0
        self.utterance_sz = 300  # utterance data to record (data in 10 ms), utterance -> 5sec
        self.record_flag = False
        self.recorded_audio = ''
        self.recorded_numpy = None
        self.recorded = False
        self.sox_transformer = sox.Transformer().set_output_format(channels=1, rate=16000, bits=16)
        rospy.init_node('iccs_audio_recorder', anonymous=True)
        rospy.Subscriber("/kinect1/audiorecord", String, self.handle_audio)
        rospy.Subscriber('/iccs/gto/controller_commands', String, self.handle_gto_controller)
        # rospy.Subscriber('/iccs/states', String, self.handle_states)

    def handle_gto_controller(self, state):
        if state.data == 'asr.listen.start':
            self.record_flag = True
            rospy.sleep(1)

    # def handle_states(self, state):
    #     if state.data == 'asr.listen':
    #         self.record_flag = True
    #         rospy.sleep(1)

    def handle_audio(self, data):
        if self.record_flag:
            if self.utterance_count < self.utterance_sz:
                self.utterance += data.data
                self.utterance_count += 1
            else:
                with contextlib.closing(wave.open(br_config.RECORDED_INPUT_RAW, 'wb')) as f:
                    f.setparams((4, 4, 16000, 0, 'NONE', 'NONE'))
                    f.writeframes(self.utterance)
                self.sox_transformer.build(br_config.RECORDED_INPUT_RAW, br_config.RECORDED_INPUT)
                with open(br_config.RECORDED_INPUT, 'rb') as speech:
                    sr, s = wavfile_read(speech)
                    self.recorded_numpy = s.copy()
                    # keep only first channel
                    if s.ndim > 1:
                        s = np.ascontiguousarray(s[:, 0])
                    self.recorded_audio = base64.b64encode(s)
                self.recorded = True
                self.utterance_count = 0
                self.utterance = ''
                self.record_flag = False
                # rospy.loginfo('Recorded audio: {}'.format(br_config.RECORDED_INPUT))

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.recorded:
                self.pub.publish(self.recorded_audio)
                self.pub_np.publish(self.recorded_numpy)
                self.recorded = False
                self.record_flag = False
            r.sleep()


if __name__ == '__main__':
    AudioRecorder().run()
