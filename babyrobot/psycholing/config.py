import babyrobot.lib.config as br_config
import os


class ROS_CONFIG(object):
    PSY_SERVICE_NAME = 'psycholing'
    PSY_SERVER_NODE = 'psycholing_server'
    PSY_CLIENT_NODE = 'psycholing_client'


PSYCHOLING_DATA = os.path.join(br_config.BASE_PATH, 'models/psycholing_data/lex')

PSY_LEXICON = os.path.join(PSYCHOLING_DATA, 'PsycholinguisticLexicon.txt')
PSY_DIMENSIONS = os.path.join(PSYCHOLING_DATA, 'PsycholinguisticDimensions.txt')