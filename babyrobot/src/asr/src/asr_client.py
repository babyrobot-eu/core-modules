#!/usr/bin/env python

import rospy

from babyrobot_msgs.msg import AudioSegment
from babyrobot_msgs.srv import SpeechRecognition


def asr(metadata):
    rospy.wait_for_service('asr')
    try:
        asr_caller = rospy.ServiceProxy('asr', SpeechRecognition)
        audio_segment = AudioSegment()
        asr_response = asr_caller(audio_segment, metadata)
        return asr_response.recognized
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node('asr_client')
    metadata = "Hello from asr client"
    recognized = asr(metadata)
    rospy.logerr("Service responded with {}".format(recognized.header.id))
