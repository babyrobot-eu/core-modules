import os

from babyrobot.lib import config as br_config

DB = os.path.join(br_config.BASE_PATH, 'db')

ROS_MODULES = os.path.join(br_config.BASE_PATH, 'babyrobot/src/')
OBJECTREC_CLIENT = os.path.join(
    ROS_MODULES, 'objectrec/src/objectrec_client.py')
EMOREC_CLIENT = os.path.join(
    ROS_MODULES, 'emorec/src/emorec_client.py')
SPEECH_FEAT_CLIENT = os.path.join(
    ROS_MODULES, 'speech_features/src/speech_feature_client.py')
CONCEPT_SPACE_CLIENT = os.path.join(
    ROS_MODULES, 'concept_net/src/concept_space_client.py')
EMBEDDINGS_FUSION_CLIENT = os.path.join(
    ROS_MODULES, 'concept_net/src/embeddings_fusion_client.py')
SEMANTIC_SIMILARITY_CLIENT = os.path.join(
    ROS_MODULES, 'concept_net/src/semantic_similarity_client.py')
