from __future__ import division

import torch
from babyrobot.emorec_pytorch.model.utilities import progress, \
    torch2numpy
from torch.autograd import Variable
from torch.nn.utils import clip_grad_norm


def train(model, dataloader, optimizer, continuous_loss, categorical_loss,
          epoch):
    model.train()
    running_loss = 0.0

    for i_batch, batch in enumerate(dataloader, 1):

        # convert to Variables
        batch = map(lambda x: Variable(x), batch)

        # convert to CUDA
        if torch.cuda.is_available():
            batch = map(lambda x: x.cuda(), batch)

        inputs, continuous, classes, lengths, indices = batch

        # 1 - zero the gradients
        optimizer.zero_grad()

        # 2 - forward pass: compute predicted y by passing x to the model
        outputs_categorical, outputs_continuous = model(inputs, lengths)

        # 3 - compute loss
        loss_continuous = continuous_loss(outputs_continuous, continuous)
        loss_categorical = categorical_loss(outputs_categorical, classes)

        # 4 - backward pass: compute gradient wrt model parameters
        loss = loss_categorical + 4 * loss_continuous
        loss.backward()

        # `clip_grad_norm` helps prevent the exploding gradient problem in RNNs
        clip_grad_norm(model.parameters(), 1)

        # 5 - update weights
        optimizer.step()

        running_loss += loss.data[0]

        # print statistics
        progress(loss=loss.data[0],
                 epoch=epoch,
                 batch=i_batch,
                 batch_size=dataloader.batch_size,
                 dataset_size=len(dataloader.dataset))

    return running_loss / i_batch


def eval(dataloader, model, categorical_loss, continuous_loss):
    """
    Perform a pass on the model and obtain the predictions, on a given dataset

    Args:
        dataloader (torch.nn.Dataloader): the dataset that will be used for
            evaluation
        model (torch.nn.Module): a pytorch model
        cat_loss ():

    Returns:

    """
    # switch to eval mode
    model.eval()

    y_cat_hat = []
    y_cat = []

    y_cont_hat = []
    y_cont = []

    total_loss = 0
    for i_batch, batch in enumerate(dataloader):

        # convert to Variables
        batch = map(lambda x: Variable(x), batch)

        # convert to CUDA
        if torch.cuda.is_available():
            batch = map(lambda x: x.cuda(), batch)

        inputs, continuous, classes, lengths, indices = batch

        # forward pass
        outputs_categorical, outputs_continuous = model(inputs, lengths)

        # compute loss
        loss_continuous = continuous_loss(outputs_continuous, continuous)
        loss_categorical = categorical_loss(outputs_categorical, classes)
        loss = loss_categorical + 4 * loss_continuous
        total_loss += loss.data[0]

        _, predicted = torch.max(outputs_categorical.data, 1)

        # cat_outputs = torch2numpy(outputs_categorical)

        y_cat.extend(list(torch2numpy(classes).squeeze()))
        y_cat_hat.extend(list(predicted))
        y_cont.extend(list(torch2numpy(continuous)))
        y_cont_hat.extend(list(torch2numpy(outputs_continuous)))

    avg_loss = total_loss / (i_batch + 1)

    return avg_loss, (y_cat, y_cat_hat), (y_cont, y_cont_hat)
