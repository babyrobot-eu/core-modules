import os

import babyrobot.lib.config as br_config


class MODELS(object):
    PATH = os.path.join(br_config.BASE_PATH, 'models/emotion')
    ZOO = {
        'anger': {
            'path': os.path.join(PATH, 'anger.model'),
            'classifier': 'weka.classifiers.functions.SMO',
            'opensmile_config': 'utterance_level',
            'classes': os.path.join(PATH, 'anger.classes')
        },
        'abc_emobase': {
            'path': os.path.join(PATH, 'abc.emobase.model'),
            'classifier': 'libsvm',
            'opensmile_config': 'emobase',
            'scale': os.path.join(PATH, 'abc.emobase.scale'),
            'classes': os.path.join(PATH, 'abc.emobase.classes'),
        },
        'avic_emobase': {
            'path': os.path.join(PATH, 'avic.emobase.model'),
            'classifier': 'libsvm',
            'opensmile_config': 'emobase',
            'scale': os.path.join(PATH, 'avic.emobase.scale'),
            'classes': os.path.join(PATH, 'avic.emobase.classes'),
        },
        'emodb_emobase': {
            'path': os.path.join(PATH, 'emodb.emobase.model'),
            'classifier': 'libsvm',
            'opensmile_config': 'emobase',
            'scale': os.path.join(PATH, 'emodb.emobase.scale'),
            'classes': os.path.join(PATH, 'emodb.emobase.classes'),
        },
        'sal_emobase': {
            'path': os.path.join(PATH, 'sal.emobase.model'),
            'classifier': 'libsvm',
            'opensmile_config': 'emobase',
            'scale': os.path.join(PATH, 'sal.emobase.scale'),
        },
        'sal_aro_emobase': {
            'path': os.path.join(PATH, 'sal_aro.emobase.model'),
            'classifier': 'libsvm',
            'opensmile_config': 'emobase',
            'scale': os.path.join(PATH, 'sal_aro.emobase.scale'),
        },
        'sal_val_emobase': {
            'path': os.path.join(PATH, 'sal_val.emobase.model'),
            'classifier': 'libsvm',
            'opensmile_config': 'emobase',
            'scale': os.path.join(PATH, 'sal_val.emobase.scale'),
        }
    }


class WEKA(object):
    PATH = '/opt/weka-3-8/weka/'
    CLASSPATH = '{}:{}'.format(
        os.path.join(PATH, 'weka.jar'),
        os.path.join(PATH, 'weka-src.jar'))
    TEMP_ARFF_FILE = '/tmp/features.arff'


class ROS_CONFIG(object):
    SERVICE_NAME = 'emorec'
    SERVER_NODE = 'emorec_server'
    CLIENT_NODE = 'emorec_client'
