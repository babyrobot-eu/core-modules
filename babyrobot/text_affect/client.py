import uuid
import rospy

from babyrobot.text_affect import config as ta_config
from babyrobot_msgs.msg import TextAffectInput
from babyrobot_msgs.srv import TextAffect


def get_text_valence(text):
    '''
    Make a call to the text_affect server.
    '''
    try:
        rospy.wait_for_service(ta_config.ROS_CONFIG.TEXT_AFFECT_SERVICE_NAME)
        if rospy.get_name() == '/unnamed':
            rospy.init_node(ta_config.ROS_CONFIG.TEXT_AFFECT_CLIENT_NODE)
        caller = rospy.ServiceProxy(
            ta_config.ROS_CONFIG.TEXT_AFFECT_SERVICE_NAME,
            TextAffect)
        query = TextAffectInput()
        query.header.id = str(uuid.uuid1())
        query.header.timestamp = rospy.Time.now()
        query.text = text
        metadata = ''
        affect_res = caller(query, metadata)
        return psycholing_res.affect
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
        return None
