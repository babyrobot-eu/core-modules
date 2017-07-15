#!/usr/bin/env python
import rospy

from babyrobot_msgs.msg import SpeechFeatures
from babyrobot_msgs.srv import SpeechFeatureExtraction
from babyrobot_msgs.srv import SpeechFeatureExtractionResponse


def handle_speech_features(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))
    msg = SpeechFeatures()
    msg.header.id = "Response sent from Speech Features service"
    return SpeechFeatureExtractionResponse(msg)


def speech_features_server():
    rospy.init_node('speech_features_server')
    rospy.Service('speech_features',
                  SpeechFeatureExtraction,
                  handle_speech_features)
    rospy.loginfo("Speech Features server started.")
    rospy.spin()


if __name__ == "__main__":
    speech_features_server()
