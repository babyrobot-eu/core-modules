#!/usr/bin/env python

import rospy

from babyrobot.asr import client as asr_client
from babyrobot.asr import config as asr_config
from babyrobot.lib.utils import mock_audio_segment

if __name__ == "__main__":
    rospy.init_node(asr_config.ROS_CONFIG.CLIENT_NODE)
    clip = mock_audio_segment(asr_config.TEST.WAV_SAMPLE)
    recognized = asr_client.asr(clip)
    rospy.loginfo(recognized)
    # rospy.logerr("Service responded with {}".format(recognized.header.id))
