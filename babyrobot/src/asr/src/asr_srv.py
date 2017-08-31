#!/usr/bin/env python
import rospy
import uuid

from babyrobot.asr import config as asr_config
from babyrobot.asr import utils as asr_utils
from babyrobot.asr import kaldi_gst_client

from babyrobot_msgs.msg import ASR
from babyrobot_msgs.srv import SpeechRecognition
from babyrobot_msgs.srv import SpeechRecognitionResponse


def handle_asr(req):
    # rospy.loginfo('Request metadata: {}'.format(req.metadata))
    asr_utils.write_wav(req.audio_segment.clip, asr_config.TEMP_FILE.WAV)
    transcription = kaldi_gst_client.run_gst(
            server_url=asr_config.KALDI_GST_SERVER.URL_DOCKER,
            clip_path=asr_config.TEMP_FILE.WAV, byte_rate=32000)
    print transcription
    msg = ASR()
    msg.header.id = str(uuid.uuid1())
    msg.related_segment_id = req.audio_segment.header.id
    msg.header.timestamp = rospy.Time.now()
    msg.transcription = transcription
    return SpeechRecognitionResponse(msg)


def asr_server():
    rospy.init_node(asr_config.ROS_CONFIG.SERVER_NODE)
    rospy.Service(
        asr_config.ROS_CONFIG.SERVICE_NAME,
        SpeechRecognition, handle_asr)
    rospy.loginfo("Speech Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    asr_server()
