class KALDI_GST_SERVER:
    PATH = ('/home/nick/Programs/kaldi/kaldi-gstreamer-server/'
            'kaldigstserver')
    URL_LOCAL = 'ws://localhost:8888/client/ws/speech?'
    URL_DOCKER = 'ws://localhost:8080/client/ws/speech?'


class KALDI_MODEL:
    PATH = ('/home/nick/Programs/kaldi/kaldi-gstreamer-server/'
            'test/models/english/voxforge/tri2b_mmi_b0.05')


class TEST:
    WAV_SAMPLE = ('/home/nick/Programs/kaldi/kaldi-gstreamer-server/'
                  'test/data/english_test.wav')


class TEMP_FILE:
    WAV = '/tmp/current.wav'


class ROS_CONFIG:
    SERVICE_NAME = 'asr'
    SERVER_NODE = 'asr_server'
    CLIENT_NODE = 'asr_client'
