import uuid
import rospy

from babyrobot.speech_features import config as sf_config
from babyrobot_msgs.msg import AudioSegment
from babyrobot_msgs.srv import SpeechFeatureExtraction


def extract_speech_features(clip,
                            opensmile_config='emobase',
                            response_format=''):
    '''
    Make a call to the speech features extraction server.
    Args:
        clip: The audio clip for which we want to extract the
            features as a hex string.
        opensmile_config: The OpenSmile configuration to use for
            the feature extraction. Current supported configurations:
            ['emobase', 'utterance_level'].
        response_format: The format of the extracted features. One of
            ['arff', 'list', '']. If the response format is 'list' or ''
            the features will be returned as a list of Feature ROS messages,
            else they will be returned as an arff file read into a string.

    Returns:
        A babyrobot_msgs/SpeechFeatures ROS message with the following fields:
        babyrobot_msgs/Header header
            string id
            time timestamp
            string metadata
        string related_segment_id
        babyrobot_msgs/Feature[] features # if response_format == 'list' or ''
            float64 feature_value
            string feature_name
        string arff_file # if response_format == 'arff'
    '''
    try:
        rospy.wait_for_service(sf_config.ROS_CONFIG.SERVICE_NAME)
        if rospy.get_name() == '/unnamed':
            rospy.init_node(sf_config.ROS_CONFIG.CLIENT_NODE)
        caller = rospy.ServiceProxy(
            sf_config.ROS_CONFIG.SERVICE_NAME,
            SpeechFeatureExtraction)
        audio_segment = AudioSegment()
        audio_segment.header.id = str(uuid.uuid1())
        audio_segment.header.timestamp = rospy.Time.now()
        audio_segment.clip = clip
        metadata = ''
        speech_feature_response = caller(audio_segment,
                                         opensmile_config,
                                         response_format,
                                         metadata)
        return speech_feature_response.extracted
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
        return None
