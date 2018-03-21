import babyrobot.lib.config as br_config
import os


class ROS_CONFIG(object):
    TEXT_AFFECT_SERVICE_NAME = 'text_affect'
    TEXT_AFFECT_SERVER_NODE = 'text_affect_server'
    TEXT_AFFECT_CLIENT_NODE = 'text_affect_client'


# TODO: Add lexicon
AFFECTIVE_DATA = os.path.join(br_config.BASE_PATH, 'models/affective_data')

AFFECTIVE_LEXICON = os.path.join(AFFECTIVE_DATA, 'THE_LEXICON.txt')
