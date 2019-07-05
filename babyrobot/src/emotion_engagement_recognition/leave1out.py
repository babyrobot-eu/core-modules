from __future__ import division
import os
import sys
import random
import argparse
import time
from shutil import copyfile
import numpy as np
from scipy.signal import medfilt
from scipy import stats
import torch.nn as nn
import torch.optim as optim
import torch.utils.data
from torch.utils.data import DataLoader
from torch.autograd import Variable
from sklearn.metrics import mean_squared_error, confusion_matrix
from dataset_skeleton import EngagementDataset, load_dataset
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
    print(args.__dict__)

    seed = 41
    np.random.seed(seed)

    seg_len = args.seg_len
    seg_overlap = args.seg_overlap
    data_path = os.path.realpath('../TrainData')
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

    num_files = len(os.listdir(data_path))

    dataset_loader = load_dataset(data_path, seg_len, seg_overlap, sequence_length, 0.2, num_classes, cuda)

    label_weights = next(dataset_loader)
    print(label_weights, torch.tensor(label_weights))

    one_hot_int = args.one_hot
    one_hot = one_hot_int == 1
    if one_hot:
        loss_function = nn.CrossEntropyLoss(weight=torch.tensor(label_weights).float())
    else:
        loss_function = WeightedMSELoss(label_weights)
    # loss_function = nn.MSELoss()
    if cuda:
        loss_function = loss_function.cuda()

    output_dir = '../output_table3/' + '_'.join(['{}' for x in [list(args.__dict__.values()) + [time.time()]][0]])
    output_dir = output_dir.format(*([list(args.__dict__.values()) + [time.time()]][0]))

    # copyfile('net.py', os.path.join(output_dir,'net.py'))
    # copyfile('leave1out.py', os.path.join(output_dir,'leave1out.py'))
    # copyfile('dataset_skeleton.py', os.path.join(output_dir,'dataset_skeleton.py'))

    total_test_loss = 0
    total_test_results = None
    total_test_targets = None
    test_results_dict = dict()
    cv_index = 0
    all_cnfs = []
    all_outputs = []
    for train_dataset, valid_dataset, test_dataset in dataset_loader:
        train_generator = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        valid_generator = DataLoader(valid_dataset, batch_size=batch_size, shuffle=True)
        test_generator = DataLoader(test_dataset, batch_size=1, shuffle=False)
        # print len(train_generator), len(valid_generator), len(test_generator)

        model = MyNet(num_classes, 2 * (58 - 12), sequence_length, hidden_size, one_hot, cuda)
        if cuda:
            model = model.cuda()
        early_stopping = EarlyStopping(patience=patience)
        early_stopping.save_weights(model)
        optimizer = optim.SGD(model.parameters(), lr=0.02, momentum=momentum, weight_decay=weight_decay)
        for stage in [0, 1]:
            # print('Stage: {}'.format(stage))
            model.change_net(stage)
            if stage == 0:
                lrs = [initial_lr, initial_lr / 10]
            else:
                lrs = [initial_lr, initial_lr / 10]
            for lr in lrs:
                early_stopping.reset()
                # print(lr)
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
                        # calc_gradients(model.parameters())
                        model.grad_clip(0.1)
                        # print('Train loss: ', loss)
                        # print(output.size(), targets.size())
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
                    # print 'Valid loss: ', total_valid_loss
                    if early_stopping.step(total_valid_loss, model):
                        early_stopping.load_weights(model)
                        break

        test_loss = 0
        test_results = None
        test_targets = None
        for i, data in enumerate(test_generator):
            input, targets = Variable(data['data']), Variable(data['target'])
            # plt.figure()
            # print targets.size()
            # plt.plot(input[0,0,:].squeeze().cpu().numpy())
            # plt.show()
            output = model(input)
            print(output.size())
            all_outputs.append(output.cpu().detach().numpy())
            if one_hot:

                max_val, max_ind = output.max(1)
                if test_results is None:
                    test_results = max_ind.data.cpu().numpy().transpose()
                    test_targets = (num_classes - 1) * targets.data.cpu().numpy().transpose()
                else:
                    test_results = np.append(test_results, max_ind.data.cpu().numpy())
                    test_targets = np.append(test_targets, (num_classes - 1) * targets.data.cpu().numpy())
                loss = loss_function(output.transpose(1, 2).view(-1, num_classes),
                                     (targets * (num_classes - 1) + 0.02).long().view(-1))
            else:
                if test_results is None:
                    test_results = (num_classes - 1) * output.data.cpu().numpy().transpose()
                    test_targets = (num_classes - 1) * targets.data.cpu().numpy().transpose()
                else:
                    test_results = np.append(test_results, (num_classes - 1) * output.data.cpu().numpy())
                    test_targets = np.append(test_targets, (num_classes - 1) * targets.data.cpu().numpy())
                # test_results.append(3*output.data.cpu().numpy()[0,0])
                # test_targets.append(3*targets.data.cpu().numpy()[0,0])
                loss = loss_function(output.squeeze(2) * (num_classes - 1), targets * (num_classes - 1))
            test_loss += loss.item() * input.shape[0] / len(test_dataset)
        # print 'Test loss: ', test_loss, mean_squared_error(medfilt(np.round(np.array(test_results)), median_filter_size), np.round(np.array(test_targets))), len(test_dataset)
        # test_results_dict[(os.listdir(data_path)[0:3]+os.listdir(data_path)[4:13]+os.listdir(data_path)[14:])[cv_index]] = test_loss
        test_results_dict[os.listdir(data_path)[cv_index]] = test_loss
        total_test_loss += mean_squared_error(np.round(np.array(test_targets)),
                                              medfilt(np.round(np.array(test_results)), median_filter_size)) / num_files

        test_results = medfilt(np.round(np.array(test_results)), median_filter_size)
        test_targets = np.round(np.array(test_targets))
        cnf = confusion_matrix(test_targets, test_results)
        save_cnf(cnf, os.path.join(output_dir, 'each_fold', 'cfn{}.txt'.format(cv_index)))
        save_vec(test_targets, os.path.join(output_dir, 'each_fold', 'targets{}.txt'.format(cv_index)))
        save_vec(test_results, os.path.join(output_dir, 'each_fold', 'results{}.txt'.format(cv_index)))
        if total_test_results is None:
            total_test_results = test_results
            total_test_targets = test_targets
        else:
            total_test_results = np.append(total_test_results, test_results, axis=0)
            total_test_targets = np.append(total_test_targets, test_targets, axis=0)

        # test_results = stats.mode(np.reshape(test_results, (-1, int(30/seg_len))), axis=1)[0]
        # test_targets = stats.mode(np.reshape(test_targets, (-1, int(30/seg_len))), axis=1)[0]
        # cnf = confusion_matrix(test_targets, test_results)
        # save_cnf(cnf, os.path.join(output_dir, 'each_fold', 'cfn{}_1sec.txt'.format(cv_index)))
        # save_vec(test_targets, os.path.join(output_dir,'each_fold','targets{}_1sec.txt'.format(cv_index)))
        # save_vec(test_results, os.path.join(output_dir,'each_fold','results{}_1sec.txt'.format(cv_index)))
        cv_index += 1
        # total_test_loss += test_loss/num_files

    import pickle
    pickle.dump(all_outputs, open('all_outputs_tmp.pkl','wb'))
    pickle.dump(total_test_targets, open('all_targets_tmp.pkl','wb'))
    exit()
    # print total_test_targets, np.unique(total_test_targets)
    # print total_test_results, np.unique(total_test_results)
    total_test_results = np.clip(total_test_results, 0, (num_classes - 1))
    # print total_test_loss, mean_squared_error(total_test_targets, total_test_results), f1_score(total_test_targets.astype(int), total_test_results.astype(int), average=None)

    # print itemfreq(total_test_targets).astype(int)

    # Compute confusion matrix
    cnf_matrix = confusion_matrix(total_test_targets, total_test_results)
    cnf_norm = cnf_matrix.astype('float') / cnf_matrix.sum(axis=1)[:, np.newaxis]

    save_cnf(os.path.join(output_dir, 'cnf.txt'), cnf_matrix)
    save_cnf(os.path.join(output_dir, 'cnf_norm.txt'), cnf_norm)
    # with open(os.path.join(output_dir,'cnf.txt'),'w') as output_file:
    #     if len(cnf_matrix.shape) > 1:
    #         for i in range(cnf_matrix.shape[0]):
    #             for j in range(cnf_matrix.shape[1]):
    #                 output_file.write('{},'.format(cnf_matrix[i,j]))
    #             output_file.write('\n')
    #     else:
    #         output_file.write('{},'.format(cnf_matrix[0]))
    # with open(os.path.join(output_dir,'cnf_norm.txt'),'w') as output_file:
    #     if len(cnf_norm.shape) > 1:
    #         for i in range(cnf_norm.shape[0]):
    #             for j in range(cnf_norm.shape[1]):
    #                 output_file.write('{},'.format(cnf_norm[i,j]))
    #             output_file.write('\n')
    #     else:
    #         output_file.write('{},'.format(cnf_norm[0]))

    # print cnf_matrix
    # print cnf_norm
    # print np.mean(np.diag(cnf_matrix)/np.sum(cnf_matrix, axis=0))
    # print f1_score(total_test_targets.astype(int), total_test_results.astype(int), average=None), np.mean(f1_score(total_test_targets.astype(int), total_test_results.astype(int), average=None))
    # with open(os.path.join(output_dir,'.txt'),'w') as output_file:

    save_vec(total_test_targets, os.path.join(output_dir, 'targets_full.txt'))
    save_vec(total_test_results, os.path.join(output_dir, 'results_full.txt'))

    total_test_results = stats.mode(
        np.reshape(total_test_results[:6 * (len(total_test_results) // (int(30 / seg_len)))], (-1, int(30 / seg_len))),
        axis=1)[0]
    total_test_targets = stats.mode(
        np.reshape(total_test_targets[:6 * (len(total_test_targets) // (int(30 / seg_len)))], (-1, int(30 / seg_len))),
        axis=1)[0]
    cnf_matrix = confusion_matrix(total_test_targets, total_test_results)
    # print cnf_matrix
    save_cnf(os.path.join(output_dir, 'cnf_1sec.txt'), cnf_matrix)
    # with open(os.path.join(output_dir,'cnf_1sec.txt'),'w') as output_file:
    #     if len(cnf_matrix.shape) > 1:
    #         for i in range(cnf_matrix.shape[0]):
    #             for j in range(cnf_matrix.shape[1]):
    #                 output_file.write('{},'.format(cnf_matrix[i,j]))
    #             output_file.write('\n')
    #     else:
    #         output_file.write('{},'.format(cnf_matrix[0]))

    save_vec(total_test_targets, os.path.join(output_dir, 'targets_1sec.txt'))
    save_vec(total_test_results, os.path.join(output_dir, 'results_1sec.txt'))

    # print f1_score(total_test_targets.astype(int), total_test_results.astype(int), average=None), np.mean(f1_score(total_test_targets.astype(int), total_test_results.astype(int), average=None))

    # print test_results_dict

    print(args.__dict__)
    print(scores(cnf_matrix.astype(np.float))['mean_f_score'], scores(cnf_matrix.astype(np.float))['accuracy'],
          scores(cnf_matrix.astype(np.float))['b_accuracy'])

    # np.set_printoptions(precision=2)
    # Plot non-normalized confusion matrix
    # plt.figure()
    # plot_confusion_matrix(cnf_matrix, classes=['0','1','2','3'],
    #                      title='Confusion matrix, without normalization')
    # Plot normalized confusion matrix
    # plt.figure()
    # plot_confusion_matrix(cnf_matrix, classes=['0','1','2','3'], normalize=True,
    #                      title='Normalized confusion matrix')
    # plt.figure()
    # t = np.arange(0,total_test_results.shape[0],1)
    # plt.plot(t,total_test_results+0.1,'r',t,total_test_targets,'b')
    # plt.figure()
    # print input.size()
    # plt.plot(np.arange(input.size(2)),input[0,0,:].squeeze().cpu().numpy())
    # plt.show()
