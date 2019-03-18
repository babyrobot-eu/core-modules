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
            #ind_from = self.indices[0][0]
            #ind_to = self.indices[-1][1]
        else:
            a = random.randint(0,self.num_segments_per_sequence-1)
            ind_from = self.indices[idx][0] - a
            ind_to = self.indices[idx][1] - a
        data = self.features[ind_from:ind_to,:]
        
        #noise = torch.randn(ind_to-ind_from, self.features.shape[1]).cuda() * self.noise_var.repeat(ind_to-ind_from, 1)
        if not self.test:
            #noise = 0.1*torch.randn(1, 1).repeat(ind_to-ind_from, 14).cuda() / self.noise_var[:,0:42:3].repeat(ind_to-ind_from, 1)
            #data[:,0:42:3] += noise
            #noise = 0.02*torch.randn(1, 1).repeat(ind_to-ind_from, 14).cuda() / self.noise_var[:,1:42:3].repeat(ind_to-ind_from, 1)
            #data[:,1:42:3] += noise
            #noise = 0.1*torch.randn(1, 1).repeat(ind_to-ind_from, 14).cuda() / self.noise_var[:,2:42:3].repeat(ind_to-ind_from, 1)
            #data[:,2:42:3] += noise
            noise = 0.5*0.04*torch.randn(ind_to-ind_from, 1).repeat(1, 14).cuda() / self.noise_var[:,0:42:3].repeat(ind_to-ind_from, 1)
            data[:,0:42:3] += noise
            noise = 0.5*0.01*torch.randn(ind_to-ind_from, 1).repeat(1, 14).cuda() / self.noise_var[:,1:42:3].repeat(ind_to-ind_from, 1)
            data[:,1:42:3] += noise
            noise = 0.5*0.04*torch.randn(ind_to-ind_from, 1).repeat(1, 14).cuda() / self.noise_var[:,2:42:3].repeat(ind_to-ind_from, 1)
            data[:,2:42:3] += noise
        #data[:,:42] /= 1.025
            #(k*x-m)/std = k*f + (k-1)*m/std
            #k = 0.05*torch.randn(...).repeat(...).cuda()
            #data[:,:42] *= k
            #d = (k-1)*self.noise_mean[:,:42].repeat(ind_to-ind_from, 1) / self.noise_var[:,:42].repeat(ind_to-ind_from, 1)
            #data[:,:42] += d
            
        target = self.labels[ind_from:ind_to]
        #target = self.labels[ind_to:ind_to+1]
        return {'data': data.transpose(0,1),#[[44,90],:],
                'target': target}
        #if self.cuda:
        #    return {'data': torch.from_numpy(data).transpose(0,1).float().cuda(),
        #            'target': torch.from_numpy(target).float().cuda()}
        #else:
        #    return {'data': torch.from_numpy(data).transpose(0,1).float(),
        #            'target': torch.from_numpy(target).float()}


def load_dataset(data_path, seg_len, seg_overlap, num_segments_per_sequence, validRatio, num_classes, cuda):
    # validRatio: 
    
    files = os.listdir(data_path)
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
        keypoints = keypoints - np.matlib.repmat(nao_pos, 1, 18)
        features = np.concatenate((keypoints[:,:42], scores), axis = 1)
        print(np.unique(labels*(num_classes-1)), files[f])
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
        indices.append(new_indices)
        
        #stds[f, :] = np.std(new_features, axis=0)
    
    #print np.mean(np.mean(stds, axis=0)[:-4])
    
    #label_counts = itemfreq(all_labels).astype(int)
    #label_counts = np.array([10,253,42,25])
    #sample_weights = np.round(np.sum(label_counts)/label_counts)#label_counts[:,1])
    #print label_counts, sample_weights
    #sample_weights = np.ones(all_labels.shape)
    #for i in range(all_labels.shape[0]):
    #    sample_weights[i] = all_labels.shape[0] / label_counts[int(all_labels[i].tolist()),1]
    #sample_weights /= sample_weights.min()

    indices_per_class = [[] for k in range(num_classes)]
    for i in range(num_files):
        file_indices_per_class = [[] for k in range(num_classes)]
        for j, ind in enumerate(indices[i]):
            majority_label = int(np.round((num_classes-1)*stats.mode(all_labels[ind[0]:ind[1]])[0]))
            for loop_ind in range(1):#range(int(sample_weights[majority_label])):
                file_indices_per_class[majority_label].append((ind[0],ind[1],majority_label))
        [indices_per_class[c].append(file_indices_per_class[c]) for c in range(num_classes)]
    #for c in range(4):
    #    indices_per_class[c] = [k for k in indices_per_class[c] if len(k) > 0]
    label_counts = np.zeros(num_classes)
    for c in range(num_classes):
        sumj = 0
        for k in range(len(indices_per_class[c])):
            sumj += len(indices_per_class[c][k])
        label_counts[c] = sumj
    label_counts = itemfreq(all_labels).astype(int)[:,1]
    sample_weights = np.sum(label_counts)/label_counts#np.round(np.sum(label_counts)/label_counts)
    sample_weights /= np.min(sample_weights)
    print(label_counts, sample_weights)
    yield sample_weights
    
    scaler = preprocessing.StandardScaler().fit(all_features)
    #print scaler.mean_, scaler.var_
    noise_mean = scaler.mean_
    noise_std = np.sqrt(scaler.var_)
    #print (all_features - noise_mean) / noise_std
    all_features = scaler.transform(all_features)
    all_features = torch.from_numpy(all_features).float()
    all_labels = torch.from_numpy(all_labels).float()
    if cuda:
        all_features = all_features.cuda()
        all_labels = all_labels.cuda()
    #print start_times
    length = len(indices)
    for i in range(num_files):#range(1)
        validIndices = list()
        trainIndices = list()
        for c in range(num_classes): #For each class
            trainValidIndices = copy.deepcopy(indices_per_class[c][0:i]+indices_per_class[c][i+1:])
            #splitPoint = int((1.0-validRatio)*len(trainValidIndices))
            #random.shuffle(trainValidIndices)
            #validIndices += [item for sublist in trainValidIndices[splitPoint:] for item in sublist]
            #trainIndices += [item for sublist in trainValidIndices[:splitPoint] for item in sublist]
            trainValidIndices = [item for sublist in trainValidIndices for item in sublist]
            splitPoint = int((1.0-validRatio)*len(trainValidIndices))
            random.shuffle(trainValidIndices)
            #print 'hi ',c,len(trainValidIndices),len(trainValidIndices[splitPoint:]),len(trainValidIndices[:splitPoint])
            for loop_ind in range(1):#range(int(sample_weights[c])):
                validIndices += trainValidIndices[splitPoint:]
                trainIndices += trainValidIndices[:splitPoint]
            
        second = [(ind[0]+1, ind[1]+1) for ind in indices[i]]
        first = [(ind[0]-num_segments_per_sequence+1,ind[1]-num_segments_per_sequence+1) for ind in indices[i]]
        testIndices = [[] for k in range(len(first)*2)]
        testIndices[0::2] = first
        testIndices[1::2] = second
        #testIndices = testIndices[0::2] + testIndices[1::2]
        #print testIndices
        #exit()
        #testIndices = []
        #for j in range(start_times[i],end_times[i]-num_segments_per_sequence):
        #    testIndices.append((j, j+num_segments_per_sequence))
        trainDataset = EngagementDataset(all_features, all_labels, trainIndices, num_segments_per_sequence,noise_mean, noise_std, cuda)
        validDataset = EngagementDataset(all_features, all_labels, validIndices, num_segments_per_sequence,noise_mean, noise_std, cuda)
        testDataset = EngagementDataset(all_features, all_labels, testIndices, num_segments_per_sequence,noise_mean, noise_std, cuda, test=True)
        #print len(trainDataset.indices), len(validDataset.indices), len(testDataset.indices)
        #print trainIndices
        yield trainDataset, validDataset, testDataset
    
