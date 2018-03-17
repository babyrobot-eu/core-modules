from __future__ import division, print_function

import sys
from collections import Counter

import matplotlib.pyplot as plt
import numpy
import torch


def sort_batch(lengths):
    """
    Sort batch data and labels by length.
    Useful for variable length inputs, for utilizing PackedSequences
    Args:
        lengths (nn.Tensor): tensor containing the lengths for the data

    Returns:
        - sorted lengths Tensor
        - sort (callable) which will sort a given iterable according to lengths
        - unsort (callable) which will revert a given iterable to its
            original order

    """
    batch_size = lengths.size(0)

    sorted_lengths, sorted_idx = lengths.sort()
    _, original_idx = sorted_idx.sort(0, descending=True)
    reverse_idx = torch.linspace(batch_size - 1, 0, batch_size).long()

    sorted_lengths = sorted_lengths[reverse_idx]

    def sort(iterable):
        if iterable.is_cuda:
            return iterable[sorted_idx.cuda()][
                reverse_idx.cuda()]
        else:
            return iterable[sorted_idx][reverse_idx]

    def unsort(iterable):
        if iterable.is_cuda:
            return iterable[reverse_idx.cuda()][
                original_idx.cuda()][
                reverse_idx.cuda()]
        else:
            return iterable[reverse_idx][original_idx][reverse_idx]

    return sorted_lengths, sort, unsort


def progress(loss, epoch, batch, batch_size, dataset_size):
    count = batch * batch_size
    bar_len = 40
    filled_len = int(round(bar_len * count / float(dataset_size)))

    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    status = 'Epoch {}, Loss: {:.4f}'.format(epoch, loss)
    _progress_str = "\r \r [{}] ...{}".format(bar, status)
    sys.stdout.write(_progress_str)
    sys.stdout.flush()


def index_array(array, indices):
    return [array[i] for i in indices]


def get_class_weights(y, smooth_factor=0):
    """
    Returns the normalized weights for each class based on the frequencies
    of the samples
    :param smooth_factor: factor that smooths extremely uneven weights
    :param y: list of true labels (the labels must be hashable)
    :return: dictionary with the weight for each class
    """
    counter = Counter(y)

    if smooth_factor > 0:
        p = max(counter.values()) * smooth_factor
        for k in counter.keys():
            counter[k] += p

    majority = max(counter.values())

    return {cls: float(majority / count) for cls, count in counter.items()}


def class_weigths(targets):
    w = get_class_weights(targets)
    labels = numpy.unique(targets)
    return torch.FloatTensor([w[l] for l in sorted(labels)])


def plot_dict(d):
    (keys, values) = zip(*d.iteritems())
    y_pos = numpy.arange(len(keys))
    plt.bar(y_pos, values, color='b')
    plt.xticks(y_pos, keys)
    plt.show()


def dataset_perf(results, metrics):
    val_loss, (y_cat, y_cat_hat), (y_cont, y_cont_hat) = results

    print("\t{}={:.4f}".format("loss", val_loss), end=", ")

    # log scores
    scores = {}
    scores.update({name: metric(y_cat, y_cat_hat)
                   for name, metric in metrics["cat"].items()})
    scores.update({name: metric(y_cont, y_cont_hat)
                   for name, metric in metrics["cont"].items()})
    for score_name, score in scores.items():
        print("{}={:.4f}".format(score_name, score), end=", ")
    print()
    return results


def torch2numpy(pt):
    try:
        return pt.data.cpu().numpy()
    except Exception:
        return pt.cpu().numpy()
