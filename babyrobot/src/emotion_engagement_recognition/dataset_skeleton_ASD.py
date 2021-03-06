from __future__ import division
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as data
from torch.autograd import Variable
import numpy as np
import scipy.io as sio
from scipy import stats
import numpy.matlib
import os
import csv
import math, random
import pandas as pd
from scipy.stats import itemfreq
from sklearn.model_selection import KFold
from sklearn import preprocessing
import copy
import matplotlib.pyplot as plt


def split_overlap(X, N, P):
    # X: input L x D
    # N: output length
    # P: overlap
    # returns K x N x D, where K is the number of segments extracted
    L = X.shape[0]
    if len(X.shape) == 1:
        D = 1
        X = np.expand_dims(X, 1)
    else:
        D = X.shape[1]
    K = (L - N) // (N - P) + 1
    Y = np.zeros((K, N, D))
    for i in range(K):
        Y[i, :, :] = X[i * (N - P):i * (N - P) + N, :]
    if Y.shape[2] == 1:
        Y = np.reshape(Y, (K, N))
    return Y


class EngagementDataset(data.Dataset):
    def __init__(self, features, labels, indices, num_segments_per_sequence, noise_mean, noise_var, cuda, test=False):

        self.features = features
        self.labels = labels
        self.indices = indices
        self.num_segments_per_sequence = num_segments_per_sequence
        self.test = test
        if test:
            self.length = len(indices)
        else:
            self.length = len(indices)
        self.cuda = cuda
        self.noise_mean = torch.from_numpy(np.reshape(np.array(noise_mean), (1, features.shape[1]))).float()
        self.noise_var = torch.from_numpy(np.reshape(np.array(noise_var), (1, features.shape[1]))).float()
        if cuda:
            self.noise_var = self.noise_var.cuda()
            self.noise_mean = self.noise_mean.cuda()

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        # if self.num_segments_per_sequence == 1:
        #    data = self.all_features[self.indices[idx],:]
        #    target = np.array(self.all_labels[self.indices[idx]])
        # else:
        # if self.test:
        #    data = self.features[self.indices[idx][0]:self.indices[idx][1],:]
        #    target = self.labels[self.indices[idx][0]:self.indices[idx][1]]
        #    print 'hi'
        # else:
        # data = self.features[self.indices[idx]:self.indices[idx]+self.num_segments_per_sequence,:]
        # target = np.array(self.labels[self.indices[idx]:self.indices[idx]+self.num_segments_per_sequence])
        if self.test:
            ind_from = self.indices[idx][0]
            ind_to = self.indices[idx][1]
            # ind_from = self.indices[0][0]
            # ind_to = self.indices[-1][1]
        else:
            a = random.randint(0, self.num_segments_per_sequence - 1)
            ind_from = self.indices[idx][0] - a
            ind_to = self.indices[idx][1] - a
        data = self.features[ind_from:ind_to, :]

        # noise = torch.randn(ind_to-ind_from, self.features.shape[1]).cuda() * self.noise_var.repeat(ind_to-ind_from, 1)
        if not self.test:
            if self.cuda:
                noise = 0.5 * 0.04 * torch.randn(ind_to - ind_from, 1).repeat(1, 14).cuda() / self.noise_var[:,
                                                                                              0:42:3].repeat(
                    ind_to - ind_from, 1)
                data[:, 0:42:3] += noise
                noise = 0.5 * 0.01 * torch.randn(ind_to - ind_from, 1).repeat(1, 14).cuda() / self.noise_var[:,
                                                                                              1:42:3].repeat(
                    ind_to - ind_from, 1)
                data[:, 1:42:3] += noise
                noise = 0.5 * 0.04 * torch.randn(ind_to - ind_from, 1).repeat(1, 14).cuda() / self.noise_var[:,
                                                                                              2:42:3].repeat(
                    ind_to - ind_from, 1)
                data[:, 2:42:3] += noise
            else:
                noise = 0.5 * 0.04 * torch.randn(ind_to - ind_from, 1).repeat(1, 14) / self.noise_var[:,
                                                                                              0:42:3].repeat(
                    ind_to - ind_from, 1)
                data[:, 0:42:3] += noise
                noise = 0.5 * 0.01 * torch.randn(ind_to - ind_from, 1).repeat(1, 14) / self.noise_var[:,
                                                                                              1:42:3].repeat(
                    ind_to - ind_from, 1)
                data[:, 1:42:3] += noise
                noise = 0.5 * 0.04 * torch.randn(ind_to - ind_from, 1).repeat(1, 14) / self.noise_var[:,
                                                                                              2:42:3].repeat(
                    ind_to - ind_from, 1)
                data[:, 2:42:3] += noise

        target = self.labels[ind_from:ind_to]
        # target = self.labels[ind_to:ind_to+1]
        return {'data': data.transpose(0, 1),
                'target': target}


def load_dataset(train_data_path, test_data_path, seg_len, seg_overlap, num_segments_per_sequence, validRatio,
                 num_classes, cuda):
    # validRatio: 

    files = os.listdir(train_data_path)  # [0:3]+os.listdir(data_path)[4:13]+os.listdir(data_path)[14:]
    full_files = [os.path.join(train_data_path, f) for f in files]
    num_files = len(files)
    start_times = []
    end_times = []
    indices = []

    all_labels = None
    # stds = np.zeros((num_files, 46))
    for f in range(num_files):
        data = sio.loadmat(full_files[f])
        labels, keypoints, scores, nao_pos, is_nan = np.ravel(data['labels']), data['smoothKeypoints'], data[
            'scores'], np.array(data['nao_head_pos']), data['is_nan']
        if num_classes == 3:
            labels[labels == 3] = 2
        labels = labels / (num_classes - 1)
        keypoints = keypoints - np.matlib.repmat(nao_pos, 1, 18)
        features = np.concatenate((keypoints[:, :42], scores), axis=1)
        # print(np.unique(labels * (num_classes - 1)), files[f])
        labels_split = split_overlap(labels, seg_len, seg_overlap)
        features_split = split_overlap(features, seg_len, seg_overlap)

        new_labels = stats.mode(labels_split, axis=1)[0].squeeze()
        new_features = np.concatenate((np.mean(features_split, axis=1),
                                       np.std(features_split, axis=1)), axis=1)
        #  np.max(features_split, axis=1),
        #  np.min(features_split, axis=1)), axis=1)

        if all_labels is None:
            all_labels = new_labels
            all_features = new_features
        else:
            all_labels = np.append(all_labels, new_labels, axis=0)
            all_features = np.append(all_features, new_features, axis=0)

        start_times.append(all_features.shape[0] - features_split.shape[0])
        end_times.append(all_features.shape[0])
        new_indices = []
        for i in range(num_segments_per_sequence - 1, end_times[-1] - start_times[-1] - num_segments_per_sequence,
                       2 * num_segments_per_sequence):
            # for i in range(end_times[-1]-start_times[-1]-num_segments_per_sequence+1):
            new_indices.append((start_times[-1] + i, start_times[-1] + i + num_segments_per_sequence))
        indices.append(new_indices)

        # stds[f, :] = np.std(new_features, axis=0)

    test_files = os.listdir(test_data_path)  # [0:3]+os.listdir(data_path)[4:13]+os.listdir(data_path)[14:]
    full_test_files = [os.path.join(test_data_path, f) for f in files]
    num_test_files = len(test_files)
    test_start_times = []
    test_end_times = []
    test_indices = []
    all_test_labels = None
    for f in range(num_test_files):
        data = sio.loadmat(full_files[f])
        labels, keypoints, scores, nao_pos, is_nan = np.ravel(data['labels']), data['smoothKeypoints'], data[
            'scores'], np.array(data['nao_head_pos']), data['is_nan']
        if num_classes == 3:
            labels[labels == 3] = 2
        labels = labels / (num_classes - 1)
        keypoints = keypoints - np.matlib.repmat(nao_pos, 1, 18)
        features = np.concatenate((keypoints[:, :42], scores), axis=1)
        labels_split = split_overlap(labels, seg_len, seg_overlap)
        features_split = split_overlap(features, seg_len, seg_overlap)

        new_labels = stats.mode(labels_split, axis=1)[0].squeeze()
        new_features = np.concatenate((np.mean(features_split, axis=1),
                                       np.std(features_split, axis=1)), axis=1)

        if all_test_labels is None:
            all_test_labels = new_labels
            all_test_features = new_features
        else:
            all_test_labels = np.append(all_test_labels, new_labels, axis=0)
            all_test_features = np.append(all_test_features, new_features, axis=0)

        test_start_times.append(all_test_features.shape[0] - features_split.shape[0])
        test_end_times.append(all_test_features.shape[0])
        new_indices = []
        for i in range(1, test_end_times[-1] - test_start_times[-1] - num_segments_per_sequence,
                       num_segments_per_sequence):
            new_indices.append((test_start_times[-1] + i, test_start_times[-1] + i + num_segments_per_sequence))
        test_indices.append(new_indices)

    indices_per_class = [[] for k in range(num_classes)]
    for i in range(num_files):
        file_indices_per_class = [[] for k in range(num_classes)]
        for j, ind in enumerate(indices[i]):
            majority_label = int(np.round((num_classes - 1) * stats.mode(all_labels[ind[0]:ind[1]])[0]))
            for loop_ind in range(1):
                file_indices_per_class[majority_label].append((ind[0], ind[1], majority_label))
        [indices_per_class[c].append(file_indices_per_class[c]) for c in range(num_classes)]

    label_counts = np.zeros(num_classes)
    for c in range(num_classes):
        sumj = 0
        for k in range(len(indices_per_class[c])):
            sumj += len(indices_per_class[c][k])
        label_counts[c] = sumj
    label_counts = itemfreq(all_labels).astype(int)[:, 1]
    sample_weights = np.sum(label_counts) / label_counts
    sample_weights /= np.min(sample_weights)
    yield sample_weights

    scaler = preprocessing.StandardScaler().fit(all_features)
    noise_mean = scaler.mean_
    noise_std = np.sqrt(scaler.var_)
    all_features = scaler.transform(all_features)
    all_test_feautures = scaler.transform(all_test_features)
    
    from sklearn.externals import joblib
    scaler_filename = "scaler.save"
    joblib.dump(scaler, scaler_filename) 
    print('Saved scaler')

    all_features = torch.from_numpy(all_features).float()
    all_labels = torch.from_numpy(all_labels).float()
    all_test_features = torch.from_numpy(all_test_features).float()
    all_test_labels = torch.from_numpy(all_test_labels).float()
    if cuda:
        all_features = all_features.cuda()
        all_labels = all_labels.cuda()
        all_test_features = all_test_features.cuda()
        all_test_labels = all_test_labels.cuda()

    for i in range(num_test_files):
        validIndices = list()
        trainIndices = list()
        for c in range(num_classes):  # For each class
            trainValidIndices = copy.deepcopy(indices_per_class[c])
            trainValidIndices = [item for sublist in trainValidIndices for item in sublist]
            splitPoint = int((1.0 - validRatio) * len(trainValidIndices))
            random.shuffle(trainValidIndices)
            for loop_ind in range(1):
                validIndices += trainValidIndices[splitPoint:]
                trainIndices += trainValidIndices[:splitPoint]

        testIndices = test_indices[i]
        trainDataset = EngagementDataset(all_features, all_labels, trainIndices, num_segments_per_sequence, noise_mean,
                                         noise_std, cuda)
        validDataset = EngagementDataset(all_features, all_labels, validIndices, num_segments_per_sequence, noise_mean,
                                         noise_std, cuda)
        testDataset = EngagementDataset(all_test_features, all_test_labels, testIndices, num_segments_per_sequence,
                                        noise_mean, noise_std, cuda, test=True)
        yield trainDataset, validDataset, testDataset
