#!/usr/bin/env python
import rospy

from babyrobot_msgs.msg import ASR
from babyrobot_msgs.srv import SpeechRecognition
from babyrobot_msgs.srv import SpeechRecognitionResponse


def handle_asr(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))
    msg = ASR()
    msg.header.id = "Response sent from ASR service"
    return SpeechRecognitionResponse(msg)


def asr_server():
    rospy.init_node('asr_server')
    rospy.Service('asr', SpeechRecognition, handle_asr)
    rospy.loginfo("Speech Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    asr_server()
