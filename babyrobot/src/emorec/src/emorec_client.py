#!/usr/bin/env python

import rospy

from babyrobot.emorec import client as emorec_client
from babyrobot.emorec import config as emorec_config
from babyrobot.speech_features.utils import mock_audio_segment


if __name__ == "__main__":
    rospy.init_node(emorec_config.ROS_CONFIG.CLIENT_NODE)
    clip = mock_audio_segment()
    recognized = emorec_client.emorec(clip)
    rospy.logerr("Service responded with {}".format(recognized))
