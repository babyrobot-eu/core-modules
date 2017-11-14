#!/usr/bin/env python

import rospy

from babyrobot.emorec import client as emorec_client
from babyrobot.emorec import config as emorec_config
from babyrobot.speech_features import config as sf_config
from babyrobot.lib.utils import mock_audio_segment
from rospy_message_converter import json_message_converter

import json


if __name__ == "__main__":
    rospy.init_node(emorec_config.ROS_CONFIG.CLIENT_NODE)

    # Sample wav file
    # clip = mock_audio_segment(sf_config.TEST.WAV_SAMPLE)

    # Get wav file
    clip = mock_audio_segment(sf_config.TMP.WAV_FILE)
    # Emotion recognition
    recognized = emorec_client.emorec(clip)
    json_recognized = json_message_converter.\
        convert_ros_message_to_json(recognized)
    with open('/tmp/emotions.json','w') as f:
        json.dump(json_recognized, f)
    rospy.logerr("Service responded with {}".format(recognized))
