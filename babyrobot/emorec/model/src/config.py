"""
Config file, for training models
"""


class Config(object):
    def to_dict(self):
        return {k: v for k, v in self.__class__.__dict__.items()
                if k[:2] != "__"}


class Baseline(Config):
    name = 'Baseline RNN'
    batch = 128
    epochs = 100
    rnn_size = 250
    rnn_layers = 1
    rnn_mode = "GRU"
    rnn_bidirectional = False
    simplify = True
    rnn_dropout = .5
    input_noise = .2
    input_dropout = .8
