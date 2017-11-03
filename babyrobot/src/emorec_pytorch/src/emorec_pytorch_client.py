#!/usr/bin/env python

import rospy

from babyrobot.emorec_pytorch import client as emorec_client
from babyrobot.speech_features import config as sf_config
from babyrobot.lib.utils import mock_audio_segment


if __name__ == "__main__":
    rospy.init_node("emorec_pytorch")
    clip = mock_audio_segment(sf_config.TEST.WAV_SAMPLE)
    recognized = emorec_client.emorec(clip)
    rospy.logerr("Service responded with {}".format(recognized))
