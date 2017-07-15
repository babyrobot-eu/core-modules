#!/usr/bin/env python

import rospy

from babyrobot_msgs.msg import AudioSegment
from babyrobot_msgs.srv import SpeechFeatureExtraction


def speech_features(metadata):
    rospy.wait_for_service('speech_features')
    try:
        caller = rospy.ServiceProxy(
            'speech_features', SpeechFeatureExtraction)
        audio_segment = AudioSegment()
        opensmile_conf = ""
        speech_feature_response = caller(
            audio_segment, opensmile_conf, metadata)
        return speech_feature_response.extracted
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node('speech_feature_client')
    metadata = "Hello from speech feature client"
    extracted = speech_features(metadata)
    rospy.logerr("Service responded with {}".format(extracted.header.id))
