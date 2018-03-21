"""
Config file, for training models
"""
import os

import babyrobot.lib.config as br_config


class Config(object):
    def to_dict(self):
        return {k: v for k, v in self.__class__.__dict__.items()
                if k[:2] != "__"}


class ModelBaseline(Config):
    name = 'Baseline RNN'
    opensmile_config = 'emobase'
    batch = 64
    epochs = 100
    encoder_type = "GRU"
    encoder_size = 100
    encoder_layers = 1
    encoder_dropout = .2
    bidirectional = False
    input_noise = .2
    input_dropout = .5
    attention_layers = 1
    attention_dropout = .0
    attention_activation = "tanh"


class Paths(Config):
    root = os.path.join(br_config.BASE_PATH, "babyrobot/emorec_pytorch/")
    models_root = os.path.join(br_config.BASE_PATH, "models/emotion_pytorch")
    checkpoint = models_root + "/emorec_pytorch.model"
    data_manager = models_root + "/data_manager.p"
    src = root + "model"
    iemocap = os.path.join(br_config.BASE_PATH, "models/data/IEMOCAP/")
    utterances_partial_path = "seglen_3/"
    index_partial_path = "IEMOCAP_index.txt"
    utterances_ath = os.path.join(iemocap, utterances_partial_path)
    index_file = os.path.join(iemocap, index_partial_path)


class General(Config):
    model = ModelBaseline()
    simplify = True  # whether to simplify the categorical labels
    paths = Paths()
    ##############################################
    # applicable only when simplify == True
    label_map = {
        'frustrated': "negative",
        'neutral': "neutral",
        'angry': "negative",
        'sad': "negative",
        'excited': "positive",
        'happy': "positive",
        'surprised': "negative",
        'fearful': "negative",
        'disgusted': "negative"
    }
    omit_labels = {"surprised"}
    ##############################################


class ROS_CONFIG(object):
    SERVICE_NAME = 'emorec_pytorch'
    SERVER_NODE = 'emorec_pytorch_server'
    CLIENT_NODE = 'emorec_pytorch_client'
