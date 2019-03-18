
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as data
from torch.autograd import Variable
import numpy as np
import scipy.io as sio
import numpy.matlib
import os
import csv
import math, random
import pandas as pd
from scipy.stats import itemfreq
from sklearn.model_selection import KFold
from sklearn import preprocessing
import copy


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
    

class EngagementDataset(data.Dataset):
    def __init__(self, features, labels, indices, num_segments_per_sequence, cuda, test=False):
        
        self.features = features
        self.labels = labels
        self.indices = indices
        self.num_segments_per_sequence = num_segments_per_sequence
        self.test = test
        self.length = len(indices)
        self.cuda = cuda
            
    
    def __len__(self):
        return self.length
    
    def __getitem__(self, idx):
        #if self.num_segments_per_sequence == 1:
        #    data = self.all_features[self.indices[idx],:]
        #    target = np.array(self.all_labels[self.indices[idx]])
        #else:
        #if self.test:
        #    data = self.features[self.indices[idx][0]:self.indices[idx][1],:]
        #    target = self.labels[self.indices[idx][0]:self.indices[idx][1]]
        #    print 'hi'
        #else:
            #data = self.features[self.indices[idx]:self.indices[idx]+self.num_segments_per_sequence,:]
            #target = np.array(self.labels[self.indices[idx]:self.indices[idx]+self.num_segments_per_sequence])
        if self.test:
            ind_from = self.indices[idx][0]
            ind_to = self.indices[idx][1]
        else:
            a = random.randint(0,self.num_segments_per_sequence-1)
            ind_from = self.indices[idx][0] - a
            ind_to = self.indices[idx][1] - a
        data = self.features[ind_from:ind_to,:]
        #target = self.labels[self.indices[idx][0]:self.indices[idx][1]]
        target = self.labels[ind_to:ind_to+1]
        return {'data': data.transpose(0,1),
                'target': target}
        #if self.cuda:
        #    return {'data': torch.from_numpy(data).transpose(0,1).float().cuda(),
        #            'target': torch.from_numpy(target).float().cuda()}
        #else:
        #    return {'data': torch.from_numpy(data).transpose(0,1).float(),
        #            'target': torch.from_numpy(target).float()}


def load_dataset(data_path, seg_len, seg_overlap, num_segments_per_sequence, validRatio, cuda):
    # validRatio: 
    
    files = os.listdir(data_path)
    full_files = [os.path.join(data_path, f) for f in files]
    num_files = len(files)
    start_times = []
    end_times = []
    indices = []

    all_labels = None
    for f in range(num_files):
        data = sio.loadmat(full_files[f])
        labels, features = np.ravel(data['labels'])/3, data['scores']
        print(np.unique(labels*3))
        #print(f,labels,features)
        labels_split = split_overlap(labels, seg_len, seg_overlap)
        features_split = split_overlap(features, seg_len, seg_overlap)
        
        new_labels = np.median(labels_split, axis=1)
        new_features = np.concatenate( (np.mean(features_split, axis=1),
                                        np.std(features_split, axis=1),
                                        np.max(features_split, axis=1),
                                        np.min(features_split, axis=1)), axis=1)
        
        if all_labels is None:
            all_labels = new_labels
            all_features = new_features
        else:
            all_labels = np.append(all_labels, new_labels, axis=0)
            all_features = np.append(all_features, new_features, axis=0)
        
        start_times.append(all_features.shape[0]-features_split.shape[0])
        end_times.append(all_features.shape[0])
        new_indices = []
        for i in range(num_segments_per_sequence-1, end_times[-1]-start_times[-1]-num_segments_per_sequence, 1):
        #for i in range(end_times[-1]-start_times[-1]-num_segments_per_sequence+1):
            new_indices.append((start_times[-1]+i,start_times[-1]+i+num_segments_per_sequence))
        indices.append(new_indices)
    
    label_counts = itemfreq(all_labels).astype(int)
    sample_weights = np.ones(all_labels.shape)
    for i in range(all_labels.shape[0]):
        sample_weights[i] = all_labels.shape[0] / label_counts[int(all_labels[i].tolist()),1]
    sample_weights /= sample_weights.min()
    
    scaler = preprocessing.StandardScaler().fit(all_features)
    all_features = scaler.transform(all_features)
    all_features = torch.from_numpy(all_features).float()
    all_labels = torch.from_numpy(all_labels).float()
    if cuda:
        all_features = all_features.cuda()
        all_labels = all_labels.cuda()
    #print start_times
    length = len(indices)
    for i in range(num_files):
        trainValidIndices = copy.deepcopy(indices[0:i]+indices[i+1:])
        splitPoint = int(validRatio*len(trainValidIndices))
        #while True:
        random.shuffle(trainValidIndices)
        validIndices = [item for sublist in trainValidIndices[:splitPoint] for item in sublist]
            #print validIndices
            #exit()
        #trainValidIndices = [item for sublist in indices[0:i]+indices[i+1:] for item in sublist]
        #random.shuffle(trainValidIndices)
        #trainIndices = trainValidIndices[splitPoint:]
        #validIndices = trainValidIndices[:splitPoint]
        trainIndices = [item for sublist in trainValidIndices[splitPoint:] for item in sublist]
        testIndices = indices[i]
        testIndices = []
        for j in range(start_times[i],end_times[i]-num_segments_per_sequence):
            testIndices.append((j, j+num_segments_per_sequence))
        trainDataset = EngagementDataset(all_features, all_labels, trainIndices, num_segments_per_sequence, cuda)
        validDataset = EngagementDataset(all_features, all_labels, validIndices, num_segments_per_sequence, cuda)
        testDataset = EngagementDataset(all_features, all_labels, testIndices, num_segments_per_sequence, cuda, test=True)
        #print len(trainDataset.indices), len(validDataset.indices), len(testDataset.indices)
        yield trainDataset, validDataset, testDataset
    
