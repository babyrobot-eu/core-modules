#!/usr/bin/env python
import uuid

import rospy

from babyrobot_msgs.msg import ASR
from babyrobot_msgs.srv import SpeechRecognition
from babyrobot_msgs.srv import SpeechRecognitionResponse


def handle_asr(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))

    transcription = "Hello from test ASR"

    msg = ASR()
    msg.header.id = str(uuid.uuid1())
    msg.header.timestamp = rospy.Time.now()
    msg.related_segment_id = req.audio_segment.header.id
    msg.transcription = transcription
    return SpeechRecognitionResponse(msg)


def asr_server():
    rospy.init_node('asr_server')
    rospy.Service('asr', SpeechRecognition, handle_asr)
    rospy.loginfo("Speech Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    asr_server()
