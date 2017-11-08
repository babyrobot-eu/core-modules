import uuid
import rospy

from babyrobot.emorec_pytorch import config as emorec_config
from babyrobot_msgs.msg import AudioSegment
from babyrobot_msgs.srv import SpeechEmotionRecognition


def emorec(clip):
    try:
        rospy.wait_for_service(emorec_config.ROS_CONFIG.SERVICE_NAME)
        emo_recognition = rospy.ServiceProxy(
            emorec_config.ROS_CONFIG.SERVICE_NAME,
            SpeechEmotionRecognition)
        audio_segment = AudioSegment()
        audio_segment.clip = clip
        audio_segment.header.id = str(uuid.uuid1())
        audio_segment.header.timestamp = rospy.Time.now()
        metadata = ''
        emorec_response = emo_recognition(audio_segment, metadata)
        return emorec_response.recognized
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
