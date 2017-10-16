"""
Config file, for training models
"""


class Config(object):
    def to_dict(self):
        return {k: v for k, v in self.__class__.__dict__.items()
                if k[:2] != "__"}


class Baseline(Config):
    name = 'Baseline RNN'
    batch = 64
    epochs = 100
    rnn_size = 250
    rnn_layers = 1
    rnn_mode = "LSTM"
    rnn_bidirectional = False
    rnn_dropout = .5
    input_noise = .4
    input_dropout = .5
