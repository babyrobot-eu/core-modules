from __future__ import division

import torch
from torch.autograd import Variable
from torch.nn.utils import clip_grad_norm
from utilities import sort_batch, progress


def train_epoch(model, dataloader, optimizer, continuous_loss,
                categorical_loss, epoch):
    model.train()
    running_loss = 0.0

    for i_batch, sample_batched in enumerate(dataloader, 1):

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
            inputs = Variable(inputs.cuda())
            lengths = Variable(lengths.cuda())
            labels[0] = Variable(labels[0].cuda())
            labels[1] = Variable(labels[1].cuda())

        # 1 - zero the gradients
        optimizer.zero_grad()

        # 2 - forward pass: compute predicted y by passing x to the model

        # register_vis_hooks(model)
        cat_outputs, cont_outputs = model(inputs, lengths)
        # remove_vis_hooks()
        # save_visualization("arch", format='svg')

        # 3 - compute loss
        cont_loss = continuous_loss(cont_outputs, labels[0])
        cat_loss = categorical_loss(cat_outputs, labels[1])

        # 4 - backward pass: compute gradient wrt model parameters
        loss = cat_loss + (3 * cont_loss)
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
