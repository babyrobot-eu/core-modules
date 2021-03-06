#!/usr/bin/env python

import rospy

from babyrobot.lib.utils import mock_audio_segment
from babyrobot.speech_features import client as sf_client
from babyrobot.speech_features import config as sf_config
from rospy_message_converter import json_message_converter

import json

if __name__ == "__main__":
    rospy.init_node(sf_config.ROS_CONFIG.CLIENT_NODE)

    # Wav sample for testing
    # clip = mock_audio_segment(sf_config.TEST.WAV_SAMPLE)

    # Get wav
    clip = mock_audio_segment(sf_config.TMP.WAV_FILE)
    # Speech feature extraction
    extracted = sf_client.extract_speech_features(clip, response_format='list')
    print(len([e.feature_value for e in extracted.features]))
    json_recognized = json_message_converter.\
        convert_ros_message_to_json(extracted)
    with open('/tmp/speechfeatures.json', 'w') as f:
        json.dump(json_recognized, f)
    rospy.loginfo("Service responded with {}".format(extracted))
