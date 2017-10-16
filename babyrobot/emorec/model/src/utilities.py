from __future__ import division

import sys

import matplotlib.pyplot as plt
import numpy
import torch


def sort_batch(lengths, others):
    """
    Sort batch data and labels by length
    Args:
        lengths (nn.Tensor): tensor containing the lengths for the data

    Returns:

    """
    batch_size = lengths.size(0)

    sorted_lengths, sorted_idx = lengths.sort()
    reverse_idx = torch.linspace(batch_size - 1, 0, batch_size).long()
    sorted_lengths = sorted_lengths[reverse_idx]

    return sorted_lengths, (lst[sorted_idx][reverse_idx] for lst in others)


def progress(loss, epoch, batch, batch_size, dataset_size):
    count = batch * batch_size
    bar_len = 40
    filled_len = int(round(bar_len * count / float(dataset_size)))

    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    status = 'Epoch {}, Loss: {:.4f}'.format(epoch, loss)
    _progress_str = "\r \r [{}] ...{}".format(bar, status)
    sys.stdout.write(_progress_str)
    sys.stdout.flush()


def plot_dict(d):
    (keys, values) = zip(*d.iteritems())
    y_pos = numpy.arange(len(keys))
    plt.bar(y_pos, values, color='b')
    plt.xticks(y_pos, keys)
    plt.show()


def torch2numpy(pt):
    try:
        return pt.data.cpu().numpy()
    except:
        return pt.cpu().numpy()
