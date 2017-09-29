#!/usr/bin/env python

import rospy

from babyrobot.lib.utils import mock_audio_segment
from babyrobot.speech_features import client as sf_client
from babyrobot.speech_features import config as sf_config


if __name__ == "__main__":
    rospy.init_node(sf_config.ROS_CONFIG.CLIENT_NODE)
    clip = mock_audio_segment(sf_config.TEST.WAV_SAMPLE)
    extracted = sf_client.extract_speech_features(clip)
    rospy.logerr("Service responded with {}".format(extracted))
