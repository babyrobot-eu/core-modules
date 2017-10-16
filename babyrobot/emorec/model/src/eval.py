from __future__ import division
import torch
from torch.autograd import Variable
from utilities import sort_batch, torch2numpy


def eval_dataset(dataloader, model, categorical_loss, continuous_loss):
    """
    Perform a pass on the model and obtain the predictions, on a given dataset

    Args:
        dataloader (torch.nn.Dataloader): the dataset that will be used for
            evaluation
        model (torch.nn.Module): a pytorch model
        cat_loss ():

    Returns:

    """

    # _batch_size = dataloader.batch_size
    # if batch_size is not None:
    #     dataloader.batch_size = batch_size

    # switch to eval mode
    model.eval()

    y_cat_hat = []
    y_cat = []

    y_cont_hat = []
    y_cont = []

    total_loss = 0
    for i_batch, sample_batched in enumerate(dataloader):
        # get the inputs (batch)
        inputs, labels, lengths, indices = sample_batched
        inputs = inputs.float()

        # sort batch (for handling inputs of variable length)
        lengths, (inputs, labels[0], labels[1]) = sort_batch(lengths,
                                                             (inputs,
                                                              labels[0],
                                                              labels[1]))

        # convert to CUDA Variables
        if torch.cuda.is_available():
            inputs = Variable(inputs.cuda(), volatile=True)
            lengths = Variable(lengths.cuda(), volatile=True)
            labels[0] = Variable(labels[0].cuda(), volatile=True)
            labels[1] = Variable(labels[1].cuda(), volatile=True)

        cat_outputs, cont_outputs = model(inputs, lengths)

        cont_loss = continuous_loss(cont_outputs, labels[0])
        cat_loss = categorical_loss(cat_outputs, labels[1])
        loss = cat_loss + cont_loss
        total_loss += loss.data[0]

        _, predicted = torch.max(cat_outputs.data, 1)

        cat_outputs = torch2numpy(cat_outputs)
        cont_outputs = torch2numpy(cont_outputs)
        cont_labels = torch2numpy(labels[0])
        cat_labels = torch2numpy(labels[1]).squeeze()

        y_cat.extend(list(cat_labels))
        y_cat_hat.extend(list(predicted))
        y_cont.extend(list(cont_labels))
        y_cont_hat.extend(list(cont_outputs))

    avg_loss = total_loss / (i_batch + 1)

    return avg_loss, (y_cat, y_cat_hat), (y_cont, y_cont_hat)
