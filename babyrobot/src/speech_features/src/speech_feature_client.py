#!/usr/bin/env python

import rospy

from babyrobot.speech_features import client as sf_client
from babyrobot.speech_features import config as sf_config
from babyrobot.speech_features import utils as sf_utils


if __name__ == "__main__":
    rospy.init_node(sf_config.ROS_CONFIG.CLIENT_NODE)
    clip = sf_utils.mock_audio_segment()
    extracted = sf_client.extract_speech_features(clip)
    rospy.logerr("Service responded with {}".format(extracted))
