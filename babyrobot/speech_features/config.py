from babyrobot.lib import config as br_config
import os


class OPENSMILE(object):
    ROOT_DIR = br_config.OPENSMILE_PATH
    CONFIG_DIR = os.path.join(br_config.BASE_PATH, 'opensmile_config')


class TEMP_FILE(object):
    WAV = '/tmp/current.wav'
    CSV_OUTPUT = '/tmp/features.csv'
    ARFF_OUTPUT = '/tmp/features.arff'


class TEST(object):
    WAV_SAMPLE = os.path.join(br_config.BASE_PATH,
                              'babyrobot/src/speech_features/test.wav')

class TMP(object):
    WAV_FILE = '/tmp/wav_clip.wav'

class ROS_CONFIG(object):
    SERVICE_NAME = 'speech_feature_extraction'
    SERVER_NODE = 'speech_features_server'
    CLIENT_NODE = 'speech_features_client'
