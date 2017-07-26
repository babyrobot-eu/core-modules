#!/usr/bin/env python
import rospy
import uuid

from babyrobot.asr import config as asr_config

from babyrobot_msgs.msg import ASR
from babyrobot_msgs.srv import SpeechRecognition
from babyrobot_msgs.srv import SpeechRecognitionResponse


def handle_asr(req):
    # rospy.loginfo('Request metadata: {}'.format(req.metadata))

    transcription = "Hello from test ASR"

    msg = ASR()
    msg.header.id = str(uuid.uuid1())
    msg.related_segment_id = req.audio_segment.header.id
    msg.header.timestamp = rospy.Time.now()
    msg.transcription = transcription
    return SpeechRecognitionResponse(msg)


def asr_server():
    rospy.init_node(asr_config.ROS_CONFIG.SERVICE_NODE)
    rospy.Service(
        asr_config.ROS_CONFIG.SERVICE_NAME,
        SpeechRecognition, handle_asr)
    rospy.loginfo("Speech Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    asr_server()
