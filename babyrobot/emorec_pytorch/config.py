"""
Config file, for training models
"""
import os


class Config(object):
    def to_dict(self):
        return {k: v for k, v in self.__class__.__dict__.items()
                if k[:2] != "__"}


class ModelBaseline(Config):
    name = 'Baseline RNN'
    batch = 128
    epochs = 100
    rnn_size = 250
    rnn_layers = 1
    rnn_mode = "GRU"
    rnn_bidirectional = False
    rnn_dropout = .5
    input_noise = .2
    input_dropout = .8


class Paths(Config):
    root = "/home/christos/PycharmProjects/babyrobot-integration/babyrobot/" \
           "emorec_pytorch/"
    checkpoint = root + "/emorec.pytorch"
    iemocap = "/home/christos/datasets/IEMOCAP"
    utterances_partial_path = "Statistical_Features_for_all_IEMOCAP/seglen_3/"
    index_partial_path = "All_labels_and_features_with_ids/data/" \
                         "IEMOCAP_wavid_emotion_valence_arousal_dominance.txt"
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
