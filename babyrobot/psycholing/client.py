import uuid
import rospy

from babyrobot.psycholing import config as psy_config
from babyrobot_msgs.msg import PsycholingInput
from babyrobot_msgs.srv import Psycholing


def get_psycholing_dims(text):
    '''
    Make a call to the psycholing server.
    '''
    try:
        rospy.wait_for_service(psy_config.ROS_CONFIG.PSY_SERVICE_NAME)
        if rospy.get_name() == '/unnamed':
            rospy.init_node(psy_config.ROS_CONFIG.PSY_CLIENT_NODE)
        caller = rospy.ServiceProxy(
            psy_config.ROS_CONFIG.PSY_SERVICE_NAME,
            Psycholing)
        query = PsycholingInput()
        query.header.id = str(uuid.uuid1())
        query.header.timestamp = rospy.Time.now()
        query.text = text
        metadata = ''
        semantic_space_response = caller(query, metadata)
        return semantic_space_response.embeddings
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
        return None
