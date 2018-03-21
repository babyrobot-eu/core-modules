import babyrobot.lib.config as br_config
import os


class ROS_CONFIG(object):
    TEXT_AFFECT_SERVICE_NAME = 'text_affect'
    TEXT_AFFECT_SERVER_NODE = 'text_affect_server'
    TEXT_AFFECT_CLIENT_NODE = 'text_affect_client'


# TODO: Add lexicon
AFFECTIVE_DATA = os.path.join(br_config.BASE_PATH, 'models/affective_data')

MSOL = os.path.join(AFFECTIVE_DATA, 'MSOL-June15-09.txt')
BING_LIU_positive = os.path.join(AFFECTIVE_DATA, 'positive-words.txt')
BING_LIU_negative = os.path.join(AFFECTIVE_DATA, 'negative-words.txt')
MPQA = os.path.join(AFFECTIVE_DATA, 'subjclueslen1-HLTEMNLP05.csv')
AFFIN = os.path.join(AFFECTIVE_DATA, 'AFFIN-111.txt')
