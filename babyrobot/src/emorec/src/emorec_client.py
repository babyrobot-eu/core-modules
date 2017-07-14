#!/usr/bin/env python

import rospy

from babyrobot_msgs.msg import AudioSegment
from babyrobot_msgs.srv import SpeechEmotionRecognition


def emorec(metadata):
    rospy.wait_for_service('emorec')
    try:
        emorec = rospy.ServiceProxy('emorec', SpeechEmotionRecognition)
        audio_segment = AudioSegment()
        emorec_response = emorec(audio_segment, metadata)
        return emorec_response.recognized
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node('emorec_client')
    metadata = "Hello from emorec client"
    recognized = emorec(metadata)
    rospy.logerr("Service responded with {}".format(recognized.header.id))
