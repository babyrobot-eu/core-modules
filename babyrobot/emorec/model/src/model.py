from torch import nn
from torch.autograd import Variable
from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence


class GaussianNoise(nn.Module):
    def __init__(self, stddev, mean=.0, dynamic=False):
        """
        Gaussian Noise layer
        Args:
            stddev (float): the standard deviation of the distribution
            mean (float): the mean of the distribution
            dynamic (bool): if true, then the mean of the noise distribution
                matches the mean of the input
        """
        super(GaussianNoise, self).__init__()
        self.stddev = stddev
        self.mean = mean

    def forward(self, x):
        if self.training:
            # noise per dimension (size = size of embeddings)
            # x.mean(1).squeeze().mean(0).squeeze().data.cpu().numpy()

            noise = Variable(x.data.new(x.size()).normal_(self.mean,
                                                          self.stddev))
            return x + noise
        return x


class BaselineRNN(nn.Module):
    def __init__(self, input_size, classes, labels, **kwargs):
        super(BaselineRNN, self).__init__()

        self.rnn_size = kwargs.get("rnn_size", 128)
        self.rnn_layers = kwargs.get("rnn_layers", 1)
        self.rnn_mode = kwargs.get("rnn_mode", "LSTM")
        self.rnn_bidirectional = kwargs.get("rnn_bidirectional", False)
        self.rnn_dropout = kwargs.get("rnn_dropout", .0)
        self.input_noise = kwargs.get("input_noise", .0)
        self.input_dropout = kwargs.get("input_dropout", .0)

        self.drop_input = nn.Dropout(self.input_dropout)
        self.noise_input = GaussianNoise(self.input_noise)
        rnn_type = nn.GRU if self.rnn_mode == "GRU" else nn.LSTM
        self.rnn = rnn_type(input_size, self.rnn_size,
                            batch_first=True,
                            num_layers=self.rnn_layers,
                            dropout=self.rnn_dropout,
                            bidirectional=self.rnn_bidirectional)
        self.drop_rnn = nn.Dropout(self.rnn_dropout)

        rnn_size = self.rnn_size
        if self.rnn.bidirectional:
            rnn_size *= 2

        self.linear_cat = nn.Linear(rnn_size, classes)
        self.linear_cont = nn.Linear(rnn_size, labels)

    # def last_timestep(self, rnn, h):
    #     if rnn.bidirectional:
    #         return torch.cat((h[-2], h[-1]), 1)
    #     else:
    #         return h[-1]

    def last_timestep(self, unpacked, lengths):
        # Index of the last output for each sequence.
        idx = (lengths - 1).view(-1, 1).expand(unpacked.size(0),
                                               unpacked.size(2)).unsqueeze(1)
        return unpacked.gather(1, idx).squeeze()

    def forward(self, x, lengths):
        x = self.noise_input(x)
        x = self.drop_input(x)

        # pack the batch
        x = pack_padded_sequence(x, list(lengths.data), batch_first=True)

        out, _ = self.rnn(x)

        # unpack output - no need if we are going to use only the last outputs
        unpacked, unpacked_len = pad_packed_sequence(out, batch_first=True)

        # h.shape = (num_layers, batch_size, rnn_size)
        last = self.last_timestep(unpacked, lengths)

        last = self.drop_rnn(last)

        o_cat = self.linear_cat(last)
        o_cont = self.linear_cont(last)
        return o_cat, o_cont
