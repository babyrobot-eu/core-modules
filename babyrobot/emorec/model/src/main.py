from __future__ import print_function

import torch
from config import Baseline
from dataset import EmotionDataset, DataHolder
from eval import eval_dataset
from model import BaselineRNN
from sklearn.metrics import accuracy_score, f1_score, mean_squared_error, \
    mean_absolute_error, explained_variance_score
from torch.utils.data import DataLoader
from train import train_epoch

CHECKPOINT = "../dist/emorec.pytorch"

_config = Baseline()

# define data holder
data_holder = DataHolder()
(X_train, y_train), (X_test, y_test) = data_holder.get_split(0.2)

# define data sets
train_set = EmotionDataset(X_train, y_train, data_holder)
test_set = EmotionDataset(X_test, y_test, data_holder)

# define data loaders
dataloader_train = DataLoader(train_set, batch_size=_config.batch,
                              shuffle=True, num_workers=4)
dataloader_test = DataLoader(test_set, batch_size=_config.batch,
                             shuffle=True, num_workers=4)

model = BaselineRNN(data_holder.input_size,
                    data_holder.label_cat_encoder.classes_.size,
                    data_holder.label_cont_encoder.scale_.size,
                    **_config.to_dict())
# weights = class_weigths(train_set)

if torch.cuda.is_available():
    # recursively go over all modules
    # and convert their parameters and buffers to CUDA tensors
    model.cuda()

categorical_loss = torch.nn.CrossEntropyLoss()
continuous_loss = torch.nn.MSELoss()
parameters = filter(lambda p: p.requires_grad, model.parameters())
optimizer = torch.optim.Adam(parameters)

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
for epoch in range(1, _config.epochs + 1):
    avg_loss = train_epoch(model, dataloader_train, optimizer,
                           continuous_loss, categorical_loss, epoch)
    print()
    val_loss, (y_cat, y_cat_hat), (y_cont, y_cont_hat) = eval_dataset(
        dataloader_test, model, categorical_loss, continuous_loss)

    if best_loss == 0:
        best_loss = val_loss

    if val_loss < best_loss:
        print("Improved model! Saving checkpoint...")
        torch.save(model, CHECKPOINT)

    print("\t{}={:.4f}".format("loss", val_loss), end=", ")

    # log scores
    scores = {}
    scores.update({name: metric(y_cat, y_cat_hat)
                   for name, metric in eval_metrics["cat"].items()})
    scores.update({name: metric(y_cont, y_cont_hat)
                   for name, metric in eval_metrics["cont"].items()})
    for score_name, score in scores.items():
        print("{}={:.4f}".format(score_name, score), end=", ")
    print()
