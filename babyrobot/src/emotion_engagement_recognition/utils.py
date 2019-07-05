import numpy as np
from contextlib import contextmanager
import matplotlib.pyplot as plt
import itertools
import torch 
import torch.nn as nn

def scores(cnf):
    precision = np.diag(cnf)/np.sum(cnf, axis=0)
    recall = np.diag(cnf)/np.sum(cnf, axis=1)
    mean_precision = np.nanmean(precision)
    mean_recall = np.nanmean(recall)
    f_score = 2*precision*recall/(precision+recall)
    f_score = np.nan_to_num(f_score)
    mean_f_score = np.nanmean(f_score)
    accuracy = np.trace(cnf)/np.sum(cnf)
    #acc = np.sum(cnf[1,:])/np.sum(cnf)
    b_accuracy = np.mean(np.diag(cnf) / np.sum(cnf, axis=1))
    return locals()

def plot_confusion_matrix(cm, classes,
                          normalize=False,
                          title='Confusion matrix',
                          cmap=plt.cm.Blues):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
        print("Normalized confusion matrix")
    else:
        print('Confusion matrix, without normalization')

    print(cm)

    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    fmt = '.2f' if normalize else 'd'
    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, format(cm[i, j], fmt),
                 horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black")

    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')

class EarlyStopping(object):
    def __init__(self, mode='min', min_delta=0, patience=10):
        self.mode = mode
        self.min_delta = min_delta
        self.patience = patience
        self.best = None
        self.num_bad_epochs = 0
        self.is_better = None
        self._init_is_better(mode, min_delta)

        if patience == 0:
            self.is_better = lambda a, b: True

    def step(self, metrics, model):
        if self.best is None:
            self.best = metrics
            return False

        if np.isnan(metrics):
            return True

        if self.is_better(metrics, self.best):
            self.num_bad_epochs = 0
            self.best = metrics
            self.save_weights(model)
        else:
            self.num_bad_epochs += 1

        if self.num_bad_epochs >= self.patience:
            return True

        return False
        
    def save_weights(self, model):
        self.state_dict = model.state_dict()
        
    def load_weights(self, model):
        model.load_state_dict(self.state_dict)

    def _init_is_better(self, mode, min_delta):
        if mode not in {'min', 'max'}:
            raise ValueError('mode ' + mode + ' is unknown!')
        if mode == 'min':
            self.is_better = lambda a, best: a < best - min_delta
        if mode == 'max':
            self.is_better = lambda a, best: a > best + min_delta
            
    def reset(self):
        #self.best = None
        self.num_bad_epochs = 0


@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout


def _assert_no_grad(tensor):
    assert not tensor.requires_grad, \
        "nn criterions don't compute the gradient w.r.t. targets - please " \
        "mark these tensors as not requiring gradients"
            
class WeightedMSELoss(nn.Module):
    #MSE with a weight given to each label, specified in weights
    #Assumes M labels are integers 0 - M-1, and weights in order
    def __init__(self, weights):
        super(WeightedMSELoss, self).__init__()
        self.weights = torch.from_numpy(np.array(weights)).view(1,1,-1).float().cuda()
        
    def forward(self, input, target):
        _assert_no_grad(target)
        weight_rep = self.weights.repeat(target.size(0),target.size(1),1)
        selected_weights = torch.gather(weight_rep,2,(target).long().unsqueeze(2)).view(target.size(0),target.size(1))
        total_loss = torch.sum(selected_weights * (target-input).pow(2))/torch.sum(selected_weights)
        return total_loss


def save_cnf(cnf, path):
    # return
    with open(path, 'w') as output_file:
        if len(cnf.shape) > 1:
            for i in range(cnf.shape[0]):
                for j in range(cnf.shape[1]):
                    output_file.write('{},'.format(cnf[i, j]))
                output_file.write('\n')
        else:
            output_file.write('{},'.format(cnf[0]))


def save_vec(vec, path):
    # return
    vec = vec.squeeze()
    with open(path, 'w') as output_file:
        if len(vec.shape) >= 1:
            for i in range(vec.shape[0]):
                output_file.write('{},'.format(vec[i]))
