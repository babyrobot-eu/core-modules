import os

from babyrobot.lib import config as br_config


ROS_MODULES = os.path.join(br_config.BASE_PATH, 'babyrobot/src/')
OBJECTREC_CLIENT = os.path.join(
    ROS_MODULES, 'objectrec/src/objectrec_client.py')
EMOREC_CLIENT = os.path.join(
    ROS_MODULES, 'emorec/src/emorec_client.py')
SPEECH_FEAT_CLIENT = os.path.join(
    ROS_MODULES, 'speech_features/src/speech_feature_client.py')
