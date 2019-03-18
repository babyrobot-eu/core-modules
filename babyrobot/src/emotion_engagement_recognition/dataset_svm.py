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
        Y[i,:,:] = X[i*(N - P):i*(N-P)+N,:]
    if Y.shape[2] == 1:
        Y = np.reshape(Y, (K,N))
    return Y
    

class EngagementDataset:
    def __init__(self, features, labels, indices, num_segments_per_sequence, test=False):
        
        self.features = features
        self.labels = labels
        self.indices = indices
        self.num_segments_per_sequence = num_segments_per_sequence
        self.test = test
            
    def __len__(self):
        return 1
    
    
    def __getitem__(self, idx):
        data = self.features[self.indices,:]
            
        target = (self.labels[self.indices]*3).astype(np.int)
        #target = self.labels[ind_to:ind_to+1]
        return {'data': data,
                'target': target}
        #if self.cuda:
        #    return {'data': torch.from_numpy(data).transpose(0,1).float().cuda(),
        #            'target': torch.from_numpy(target).float().cuda()}
        #else:
        #    return {'data': torch.from_numpy(data).transpose(0,1).float(),
        #            'target': torch.from_numpy(target).float()}


def load_dataset(data_path, seg_len, seg_overlap, num_segments_per_sequence, validRatio, num_classes):
    # validRatio: 
    
    files = os.listdir(data_path)#[0:3]+os.listdir(data_path)[4:13]+os.listdir(data_path)[14:]
    full_files = [os.path.join(data_path, f) for f in files]
    num_files = len(files)
    start_times = []
    end_times = []
    indices = []

    all_labels = None
    #stds = np.zeros((num_files, 46))
    for f in range(num_files):
        data = sio.loadmat(full_files[f])
        labels, keypoints, scores, nao_pos, is_nan = np.ravel(data['labels']), data['smoothKeypoints'], data['scores'], np.array(data['nao_head_pos']), data['is_nan']
        if num_classes == 3:
            labels[labels==3] = 2
        labels = labels / (num_classes-1)
        #]print keypoints.shape, nao_pos.shape[1]
        keypoints = keypoints - np.matlib.repmat(nao_pos, 1, 18)
        features = np.concatenate((keypoints[:,:42], scores), axis = 1)
        #print np.unique(labels*(num_classes-1)), files[f]
        #print(f,labels,features)
        labels_split = split_overlap(labels, seg_len, seg_overlap)
        features_split = split_overlap(features, seg_len, seg_overlap)
        
        new_labels = stats.mode(labels_split, axis=1)[0].squeeze()
        new_features = np.concatenate( (np.mean(features_split, axis=1),
                                        np.std(features_split, axis=1)), axis=1)
                                      #  np.max(features_split, axis=1),
                                      #  np.min(features_split, axis=1)), axis=1)
        
        if all_labels is None:
            all_labels = new_labels
            all_features = new_features
        else:
            all_labels = np.append(all_labels, new_labels, axis=0)
            all_features = np.append(all_features, new_features, axis=0)
        
        start_times.append(all_features.shape[0]-features_split.shape[0])
        end_times.append(all_features.shape[0])
        new_indices = []
        for i in range(num_segments_per_sequence-1, end_times[-1]-start_times[-1]-num_segments_per_sequence, 2*num_segments_per_sequence):
        #for i in range(end_times[-1]-start_times[-1]-num_segments_per_sequence+1):
            new_indices.append((start_times[-1]+i,start_times[-1]+i+num_segments_per_sequence))
        indices.append(np.arange(start_times[-1], end_times[-1]))
        
        #stds[f, :] = np.std(new_features, axis=0)
    
    #for c in range(4):
    #    indices_per_class[c] = [k for k in indices_per_class[c] if len(k) > 0]
    label_counts = itemfreq(all_labels).astype(int)[:,1]
    sample_weights = np.sum(label_counts)/label_counts#np.round(np.sum(label_counts)/label_counts)
    #print label_counts, sample_weights
    sample_weights /= np.min(sample_weights)
    yield sample_weights
    
    scaler = preprocessing.StandardScaler().fit(all_features)
    #print scaler.mean_, scaler.var_
    noise_mean = scaler.mean_
    noise_std = np.sqrt(scaler.var_)
    #print (all_features - noise_mean) / noise_std
    all_features = scaler.transform(all_features)
    #print all_features
    #all_features += np.random.randn(*all_features.shape)*0.05
    #print np.std(all_features), np.mean(all_features)
    #normalizer = preprocessing.Normalizer().fit(all_features)
    #all_features = normalizer.transform(all_features)
    all_features = all_features.astype(np.float)
    all_labels = all_labels.astype(np.float)
    
    #print start_times
    length = len(indices)
    for i in range(num_files):#range(1)
            
        trainIndices = np.concatenate(indices[0:i]+indices[i+1:], axis=0)
        testIndices = indices[i]

        #testIndices = testIndices[0::2] + testIndices[1::2]
        #print testIndices
        #exit()
        #testIndices = []
        #for j in range(start_times[i],end_times[i]-num_segments_per_sequence):
        #    testIndices.append((j, j+num_segments_per_sequence))
        trainDataset = EngagementDataset(all_features, all_labels, trainIndices, num_segments_per_sequence)
        validDataset = EngagementDataset(all_features, all_labels, [], num_segments_per_sequence)
        testDataset = EngagementDataset(all_features, all_labels, testIndices, num_segments_per_sequence, test=True)
        #print len(trainDataset.indices), len(validDataset.indices), len(testDataset.indices)
        #print trainIndices
        yield trainDataset, validDataset, testDataset
    
