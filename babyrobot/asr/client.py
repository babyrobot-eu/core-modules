import rospy
import uuid

from babyrobot.asr import config as asr_config
from babyrobot_msgs.msg import AudioSegment
from babyrobot_msgs.srv import SpeechRecognition


def asr(clip):
    try:
        rospy.wait_for_service(asr_config.ROS_CONFIG.SERVICE_NAME)
        asr = rospy.ServiceProxy(
            asr_config.ROS_CONFIG.SERVICE_NAME,
            SpeechRecognition)
        audio_segment = AudioSegment()
        audio_segment.clip = clip
        audio_segment.header.id = str(uuid.uuid1())
        audio_segment.header.timestamp = rospy.Time.now()
        metadata = ''
        asr_response = asr(audio_segment, metadata)
        return asr_response.recognized
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))
