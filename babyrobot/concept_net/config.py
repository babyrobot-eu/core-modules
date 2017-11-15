import os
from babyrobot.lib import config as br_config


class ROS_CONFIG(object):
    SEM_SPACE_SERVICE_NAME = 'semantic_space'
    SEM_SPACE_SERVER_NODE = 'semantic_space_server'
    SEM_SPACE_CLIENT_NODE = 'semantic_space_client'
    FUSION_SERVICE_NAME = 'embeddings_fusion'
    FUSION_SERVER_NODE = 'embeddings_fusion_server'
    FUSION_CLIENT_NODE = 'embeddings_fusion_client'
    SEM_SIM_SERVICE_NAME = 'semantic_similariry'
    SEM_SIM_SERVER_NODE = 'semantic_similariry_server'
    SEM_SIM_CLIENT_NODE = 'semantic_similariry_client'


class SEMANTIC_SPACE(object):
    TEXT = os.path.join(br_config.BASE_PATH,
                        'models/semantic_spaces/text.json')
    VISUAL = os.path.join(br_config.BASE_PATH,
                          'models/semantic_spaces/visual.json')
    AUDIO = os.path.join(br_config.BASE_PATH,
                         'models/semantic_spaces/audio.json')
