import uuid
import rospy

from babyrobot.concept_net import config as cn_config
from babyrobot_msgs.msg import SemanticSpaceQuery
from babyrobot_msgs.msg import SemanticSimilarityInput
from babyrobot_msgs.msg import EmbeddingsFusionInput
from babyrobot_msgs.srv import SemanticSpace
from babyrobot_msgs.srv import EmbeddingsFusion


def get_semantic_embeddings(word):
    '''
    Make a call to the semantic space server.
    Args:
        word: The word for which we need to retrieve the semantic embeddings
    Returns:
        a babyrobot_msgs/SemanticSpaceResponse message with the
        text, visual and audio  embeddings as
        babyrobot_msgs/SemanticEmbedding messages
    '''
    try:
        rospy.wait_for_service(cn_config.ROS_CONFIG.SEM_SPACE_SERVICE_NAME)
        if rospy.get_name() == '/unnamed':
            rospy.init_node(cn_config.ROS_CONFIG.SEM_SPACE_CLIENT_NODE)
        caller = rospy.ServiceProxy(
            cn_config.ROS_CONFIG.SEM_SPACE_SERVICE_NAME,
            SemanticSpace)
        query = SemanticSpaceQuery()
        query.header.id = str(uuid.uuid1())
        query.header.timestamp = rospy.Time.now()
        query.word = word
        metadata = ''
        semantic_space_response = caller(query, metadata)
        return semantic_space_response.embeddings
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
        return None


def fuse_semantic_embeddings(word, text, visual, audio):
    '''
    Make a call to the embeddings fusion server.
    Args:
        text: The text semantic embedding as a float list
        visual: The visual semantic embedding as a float list
        audio: The audio semantic embedding as a float list
    Returns:
        a babyrobot_msgs/SemanticEmbedding message with the
        late fusion fused vector.
    '''
    try:
        rospy.wait_for_service(cn_config.ROS_CONFIG.FUSION_SERVICE_NAME)
        if rospy.get_name() == '/unnamed':
            rospy.init_node(cn_config.ROS_CONFIG.FUSION_CLIENT_NODE)
        caller = rospy.ServiceProxy(
            cn_config.ROS_CONFIG.FUSION_SERVICE_NAME,
            EmbeddingsFusion)
        query = EmbeddingsFusionInput()
        query.header.id = str(uuid.uuid1())
        query.header.timestamp = rospy.Time.now()
        query.word = word
        query.text = text
        query.visual = visual
        query.audio = audio
        metadata = ''
        print(query)
        embeddings_fusion_response = caller(query, metadata)
        return embeddings_fusion_response.fused
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
        return None


def get_semantic_similarity(v1, v2):
    '''
    Get the semantic similarity between 2 vectors v1 and v2.
    Args:
        v1: The first semantic embedding as a float list
        v2: The second semantic embedding as a float list
    Returns:
        a float that is the cosine similarity btw v1 and v2
    '''
    try:
        rospy.wait_for_service(cn_config.ROS_CONFIG.SEM_SIM_SERVICE_NAME)
        if rospy.get_name() == '/unnamed':
            rospy.init_node(cn_config.ROS_CONFIG.SEM_SIM_CLIENT_NODE)
        caller = rospy.ServiceProxy(
            cn_config.ROS_CONFIG.SEM_SIM_SERVICE_NAME,
            EmbeddingsFusion)
        embeddings = SemanticSimilarityInput()
        embeddings.header.id = str(uuid.uuid1())
        embeddings.header.timestamp = rospy.Time.now()
        embeddings.v1 = v1
        embeddings.v2 = v2
        metadata = ''
        semantic_similarity_response = caller(embeddings, metadata)
        return semantic_similarity_response.similarity_score
    except rospy.ServiceException, ex:
        rospy.logerr("Service call failed: {}".format(ex))
