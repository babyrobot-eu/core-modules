#!/usr/bin/env python
import rospy

from babyrobot_msgs.msg import EmotionRecognitionResult
from babyrobot_msgs.srv import SpeechEmotionRecognition
from babyrobot_msgs.srv import SpeechEmotionRecognitionResponse


def handle_emorec(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))
    msg = EmotionRecognitionResult()
    msg.header.id = "Response sent from EmoRec service"
    return SpeechEmotionRecognitionResponse(msg)


def emorec_server():
    rospy.init_node('emorec_server')
    rospy.Service('emorec', SpeechEmotionRecognition, handle_emorec)
    rospy.loginfo("Emotion Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    emorec_server()
