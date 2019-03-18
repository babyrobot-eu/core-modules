from __future__ import division
import os
import pickle
import sys
import random
import argparse
import time
from shutil import copyfile
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from scipy import stats
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data
from torch.utils.data import DataLoader
from torch.autograd import Variable
from sklearn import svm, linear_model, neural_network, preprocessing
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor
from sklearn.metrics import mean_squared_error, accuracy_score, f1_score, log_loss, make_scorer, confusion_matrix
from sklearn.model_selection import cross_val_score, GridSearchCV
from scipy.stats import itemfreq
# from mlens.metrics import make_scorer
# from mlens.model_selection import Evaluator
from scipy.stats import uniform, randint
from dataset_skeleton_ASD import EngagementDataset, load_dataset
from utils import suppress_stdout, EarlyStopping, plot_confusion_matrix, WeightedMSELoss, scores, save_cnf, save_vec
from net import MyNet, calc_gradients


def arg_parse():
    parser = argparse.ArgumentParser(description='Engagement estimation with LSTM')

    parser.add_argument("--seg_len", dest="seg_len", help="Segment length", default=5, type=int)
    parser.add_argument("--seg_overlap", dest="seg_overlap", help="Segment overlap", default=0, type=int)
    parser.add_argument("--seq_len", dest="seq_len", help="Number of segments per sequence", default=30, type=int)
    parser.add_argument("--median", dest="median", help="Median filter size", default=5, type=int)
    parser.add_argument("--hidden_size", dest="hidden_size", help="Hidden size", default=560, type=int)
    parser.add_argument("--initial_lr", dest="initial_lr", help="Initial learning rate", default=0.1, type=float)
    parser.add_argument("--decay", dest="decay", help="Weight decay", default=1e-6, type=float)
    parser.add_argument("--momentum", dest="momentum", help="Training momentum", default=0.5, type=float)
    parser.add_argument("--one_hot", dest="one_hot", help="Use cross entropy loss instead of MSE", default=1, type=int)
    parser.add_argument("--patience", dest="patience", help="Early stopping patience", default=10, type=int)
    parser.add_argument("--batch", dest="batch_size", help="Batch size", default=16, type=int)

    return parser.parse_args()


if __name__ == '__main__':
    args = arg_parse()

    seed = 41
    np.random.seed(seed)

    seg_len = args.seg_len
    seg_overlap = args.seg_overlap
    train_data_path = os.path.realpath('../TrainData')
    test_data_path = os.path.realpath('../TrainDataASD')
    sequence_length = args.seq_len  # Number of segments per sequence
    num_epochs = 500
    batch_size = args.batch_size
    median_filter_size = args.median
    cuda = True
    hidden_size = args.hidden_size
    initial_lr = args.initial_lr
    weight_decay = args.decay
    momentum = args.momentum
    patience = args.patience
    num_classes = 3

    num_files = len(os.listdir(test_data_path))

    for iteration in range(1):
        print('Iteration:', iteration)
        dataset_loader = load_dataset(train_data_path, test_data_path, seg_len, seg_overlap, sequence_length, 0.2,
                                      num_classes, cuda)

        label_weights = next(dataset_loader)
        one_hot_int = args.one_hot
        one_hot = one_hot_int == 1
        if one_hot:
            loss_function = nn.CrossEntropyLoss(weight=torch.tensor(label_weights).float())
        else:
            loss_function = WeightedMSELoss(label_weights)
        if cuda:
            loss_function = loss_function.cuda()

        output_dir = '../output_final_ASD/' + '_'.join(['{}' for x in [list(args.__dict__.values()) + [time.time()]][0]])
        output_dir = output_dir.format(*([list(args.__dict__.values()) + [time.time()]][0]))

        test_results_dict = dict()
        cv_index = 0
        all_cnfs = []
        loop_cnt = 0

        train_dataset, valid_dataset, _ = next(dataset_loader)
        train_generator = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        valid_generator = DataLoader(valid_dataset, batch_size=batch_size, shuffle=True)

        model = MyNet(num_classes, 2 * (58 - 12), sequence_length, hidden_size, one_hot, cuda)
        if cuda:
            model = model.cuda()
        early_stopping = EarlyStopping(patience=patience)
        early_stopping.save_weights(model)
        optimizer = optim.SGD(model.parameters(), lr=0.02, momentum=momentum, weight_decay=weight_decay)
        for stage in [0, 1]:
            print('Stage: {}'.format(stage))
            model.change_net(stage)
            lrs = [initial_lr, initial_lr / 10]
            for lr in lrs:
                early_stopping.reset()
                print(lr)
                for param_group in optimizer.param_groups:
                    param_group['lr'] = lr
                for epoch in range(num_epochs):
                    model.train()
                    for i, data in enumerate(train_generator):
                        input, targets = Variable(data['data']), Variable(data['target'])
                        model.zero_grad()
                        if input.size(0) == 1:
                            continue
                        output = model(input)
                        if one_hot:
                            loss = loss_function(output.transpose(1, 2).view(-1, num_classes),
                                                 (targets * (num_classes - 1) + 0.02).long().view(-1))
                        else:
                            loss = loss_function(output.squeeze(2) * (num_classes - 1), targets * (num_classes - 1)) / (
                                        (num_classes - 1) ** 2)
                        loss.backward()
                        model.grad_clip(0.1)
                        optimizer.step()
                    model.eval()
                    total_valid_loss = 0
                    for i, data in enumerate(valid_generator):
                        input, targets = Variable(data['data']), Variable(data['target'])
                        output = model(input)
                        if one_hot:
                            loss = loss_function(output.transpose(1, 2).view(-1, num_classes),
                                                 (targets * (num_classes - 1) + 0.02).long().view(-1))
                        else:
                            loss = loss_function(output.squeeze(2) * (num_classes - 1), targets * (num_classes - 1)) / (
                                        (num_classes - 1) ** 2)
                        total_valid_loss += loss.item() * input.shape[0] / len(valid_dataset)
                    # print('Valid loss: ', total_valid_loss)
                    if early_stopping.step(total_valid_loss, model):
                        early_stopping.load_weights(model)
                        break

        torch.save(model.state_dict(), 'final_model.torch')
        print('Model saved')
        exit()
        model.train()
        dataset_loader = load_dataset(train_data_path, test_data_path, seg_len, seg_overlap, sequence_length, 0.0,
                                      num_classes, cuda)

        label_weights = next(dataset_loader)
        train_dataset, _, _ = next(dataset_loader)
        train_generator = DataLoader(train_dataset, batch_size=len(train_dataset), shuffle=True)
        for i, data in enumerate(train_generator):
            input, targets = Variable(data['data'], requires_grad=True), Variable(data['target'])
            model.zero_grad()
            if input.size(0) == 1:
                print('Bad input :(')
            output = model(input)
            if one_hot:
                loss = loss_function(output.transpose(1, 2).view(-1, num_classes),
                                     (targets * (num_classes - 1) + 0.02).long().view(-1))
            else:
                loss = loss_function(output.squeeze(2) * (num_classes - 1), targets * (num_classes - 1)) / (
                        (num_classes - 1) ** 2)
            loss.backward(retain_graph=True)

        fea_names = []
        for t in ['m', 'v']:
            for i in range(14):
                for d in ['x', 'y', 'z']:
                    fea_names.append('k{}_{}_{}'.format(i, d, t))
            for i in range(4):
                fea_names.append('f{}_{}'.format(i, t))
        fea_names = np.array(fea_names)
        if input.grad.data is not None:
            pickle.dump(torch.mean(input.grad.data, 0).cpu().numpy(),
                        open('../pkl/input_gradients_{}.pkl'.format(iteration), 'wb'))
            # nn_weights = np.linalg.norm(torch.mean(input.grad.data, 0).cpu().numpy(), axis=1, ord=np.inf)
            # print(np.sort(nn_weights))
            # print(np.argsort(nn_weights))
            # print(fea_names[np.argsort(nn_weights)])
        else:
            print('No grad')
