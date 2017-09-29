class OPENSMILE(object):
    ROOT_DIR = '/home/geopar/projects/opensmile'
    CONFIG_DIR = '/home/geopar/projects/babyrobot-integration/opensmile_config'


class TEMP_FILE(object):
    WAV = '/tmp/current.wav'
    CSV_OUTPUT = '/tmp/features.csv'
    ARFF_OUTPUT = '/tmp/features.arff'


class TEST(object):
    WAV_SAMPLE = ('/home/geopar/projects/babyrobot-integration/babyrobot'
                  '/src/speech_features/test.wav')


class ROS_CONFIG(object):
    SERVICE_NAME = 'speech_feature_extraction'
    SERVER_NODE = 'speech_features_server'
    CLIENT_NODE = 'speech_features_client'
