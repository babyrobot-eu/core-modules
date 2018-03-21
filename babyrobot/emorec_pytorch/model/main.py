from __future__ import print_function

import pickle
from copy import deepcopy

import torch
from sklearn.metrics import accuracy_score, f1_score, mean_squared_error, \
    explained_variance_score, mean_absolute_error
from torch.utils.data import DataLoader

from babyrobot.emorec_pytorch.config import General
from babyrobot.emorec_pytorch.model.dataset import DataManager, EmotionDataset
from babyrobot.emorec_pytorch.model.modules.models import MultitaskEmotionModel
from babyrobot.emorec_pytorch.model import pipelines
from babyrobot.emorec_pytorch.model.utilities import (class_weigths,
                                                      dataset_perf)

_confing = General()

# define data holder
_data_manager = DataManager(simplify=_confing.simplify)
(X_train, y_train), (X_test, y_test) = _data_manager.split(0.1)

# define data sets
train_set = EmotionDataset(X_train, y_train, _data_manager)
test_set = EmotionDataset(X_test, y_test, _data_manager)

# define data loaders
dataloader_train = DataLoader(train_set, batch_size=_confing.model.batch,
                              shuffle=True, num_workers=4)
dataloader_test = DataLoader(test_set, batch_size=_confing.model.batch,
                             shuffle=True, num_workers=4)

_model = MultitaskEmotionModel(_data_manager.input_size,
                               _data_manager.label_cat_encoder.classes_.size,
                               _data_manager.label_cont_encoder.scale_.size,
                               **_confing.model.to_dict())
weights = class_weigths(train_set.targets[1])

if torch.cuda.is_available():
    # recursively go over all modules
    # and convert their parameters and buffers to CUDA tensors
    _model.cuda()
    weights = weights.cuda()

cat_loss = torch.nn.CrossEntropyLoss(weight=weights)
cont_loss = torch.nn.MSELoss()
parameters = filter(lambda p: p.requires_grad, _model.parameters())
optimizer = torch.optim.Adam(params=parameters,
                             weight_decay=0.001
                             )
print(_model)
#############################################################
# Experiment
#############################################################

eval_metrics = {
    "cat": {
        "acc": lambda y, y_hat: accuracy_score(y, y_hat),
        "f1_M": lambda y, y_hat: f1_score(y, y_hat, average='macro'),
    },
    "cont": {
        "mse": lambda y, y_hat: mean_squared_error(y, y_hat),
        "mae": lambda y, y_hat: mean_absolute_error(y, y_hat),
        "evs": lambda y, y_hat: explained_variance_score(y, y_hat)
    }
}

#############################################################
# Train
#############################################################
best_loss = 0
patience = 10
patience_left = patience
for epoch in range(1, _confing.model.epochs + 1):
    avg_loss = pipelines.train(_model, dataloader_train, optimizer,
                               cont_loss, cat_loss, epoch)
    print()

    # evaluate training set
    print("Training:")
    results = pipelines.eval(dataloader_train, _model, cat_loss, cont_loss)
    train_loss, _, _ = dataset_perf(results, eval_metrics)

    # evaluate validation set
    print("Validation:")
    results = pipelines.eval(dataloader_test, _model, cat_loss, cont_loss)
    val_loss, _, _ = dataset_perf(results, eval_metrics)

    # if best_loss == 0:
    #     best_loss = val_loss
    #
    # if val_loss < best_loss:
    #     best_loss = val_loss
    #     patience_left = patience
    #     print("Improved model! Saving checkpoint...")
    #     torch.save(_model, _confing.paths.checkpoint)
    #
    #     # write data_manager in order to be able to prepare new samples
    #     dm = deepcopy(_data_manager)
    #     dm.data = None
    #     dm.target = None
    #     with open(_confing.paths.data_manager, 'wb') as f:
    #         pickle.dump(dm, f, protocol=pickle.HIGHEST_PROTOCOL)
    # else:
    #     patience_left -= 1
    #
    # if patience_left == 0:
    #     break
