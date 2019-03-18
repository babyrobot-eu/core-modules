#!/usr/bin/env python
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
from scipy.stats import uniform, randint
from dataset_skeleton_ASD import EngagementDataset, load_dataset
from utils import *#suppress_stdout, EarlyStopping, plot_confusion_matrix, WeightedMSELoss, scores, save_cnf, save_vec
from net import MyNet, calc_gradients
import rospy
from std_msgs.msg import Header, Float64MultiArray, Int32
from geometry_msgs.msg import PointStamped
from threading import Thread, Lock



def arg_parse():
    parser = argparse.ArgumentParser(description='Engagement estimation with LSTM')

    parser.add_argument("--seg_len", dest="seg_len", help="Segment length", default=5, type=int)
    parser.add_argument("--seg_overlap", dest="seg_overlap", help="Segment overlap", default=0, type=int)
    parser.add_argument("--seq_len", dest="seq_len", help="Number of segments per sequence", default=30, type=int)
    parser.add_argument("--median", dest="median", help="Median filter size", default=5, type=int)
    parser.add_argument("--hidden_size", dest="hidden_size", help="Hidden size", default=560, type=int)
    parser.add_argument("--one_hot", dest="one_hot", help="Use cross entropy loss instead of MSE", default=1, type=int)

    return parser.parse_known_args()[0]


class DataSubscriber:
    """ Holds the most up to date """
    def __init__(self):
        self._sub = rospy.Subscriber('/engagement_features',
                                          Float64MultiArray,
                                          self._callback, queue_size=1, buff_size=10000000)

        # data containers and its mutexes
        self.data = []
        self.new_data = False
        self._mutex = Lock()

    def _callback(self, data):
        self._mutex.acquire()
        self.data.append(data.data)
        while len(self.data) > 5*30:
            self.data.pop(0)
        self.new_data = True
        self._mutex.release()

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

def prep_data(data, scaler, cuda):
    data = np.array(data)
    print(data.shape)
    keypoints = data[:,:54]
    scores = data[:,54:-3]
    nao_pos = data[:,-3:]
    keypoints = keypoints - np.matlib.repmat(nao_pos, 1, 18)  
    features = np.concatenate((keypoints[:,:42], scores), axis = 1)
    features_split = split_overlap(features, 5, 0)
    new_features = np.concatenate( (np.mean(features_split, axis=1),
                                        np.std(features_split, axis=1)), axis=1)
                                        
    all_features = scaler.transform(new_features)
    all_features = torch.from_numpy(all_features).float()
    if cuda:
        all_features = all_features.cuda()
    return all_features.transpose(0, 1).unsqueeze(0)
    

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
    cuda = True
    hidden_size = args.hidden_size
    one_hot_int = args.one_hot
    one_hot = one_hot_int == 1
    num_classes = 3

    num_files = 1
    
    model = MyNet(num_classes, 2 * (58 - 12), sequence_length, hidden_size, one_hot, cuda)
    model.load_state_dict(torch.load('/home/babyrobot/catkin_ws/src/engagement_emotion/scripts/final_model.torch'))
    model.eval()
    model = model.cuda()
    
    from sklearn.externals import joblib
    scaler_filename = "/home/babyrobot/catkin_ws/src/engagement_emotion/scripts/scaler.save"
    scaler = joblib.load(scaler_filename) 
    
    rospy.init_node('engagement_predictor', anonymous=True)
    sub = DataSubscriber()
    pub = rospy.Publisher('/engagement_prediction', PointStamped, queue_size=10)
    try:
        while not rospy.is_shutdown():
            sub._mutex.acquire()

            if len(sub.data) > 4 and sub.new_data == True:
                input = prep_data(sub.data, scaler, cuda)
                output = model(input)
                result = np.argmax(output.detach().cpu().numpy()[0,:,-1])
                #msg = Int32()
                #msg.data = result
                #pub.publish(msg)
                msg = PointStamped()
                msg.point.x = float(result)
                msg.header.stamp = rospy.Time.now()
                pub.publish(msg)
		print(result)
                sub.new_data = False
            
            sub._mutex.release()

    except KeyboardInterrupt:
        print("Shutting down")

