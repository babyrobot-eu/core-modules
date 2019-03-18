import torch
import torch.nn as nn
import torch.utils.data
from torch.autograd import Variable
import numpy as np

def calc_gradients(params):
    grad_array = []
    _mean = []
    _max = []
    for param in params:
        if not (param.grad is None):
            grad_array.append(param.grad.data)
            _mean.append(torch.mean(param.grad.data))
            _max.append(torch.max(param.grad.data))
    print('Mean: {}'.format(np.mean(_mean)))
    print('Max:  {}'.format(np.max(_max)))

class LambdaBase(nn.Sequential):
    def __init__(self, fn, *args):
        super(LambdaBase, self).__init__(*args)
        self.lambda_func = fn

    def forward_prepare(self, input):
        output = []
        for module in self._modules.values():
            output.append(module(input))
        return output if output else input

class Lambda(LambdaBase):
    def forward(self, input):
        return self.lambda_func(self.forward_prepare(input))

class MyNet(nn.Module):
    
    def __init__(self, num_classes, in_channels, sequence_length, hidden_size, one_hot, cuda):
        super(MyNet, self).__init__()
        
        self.iscuda = cuda
        self.sequence_length = sequence_length
        self.hidden_size = hidden_size
        self.one_hot = one_hot
        if self.one_hot:
            self.output_size = num_classes
        else:
            self.output_size = 1
        self.stage = 0
        

        self.i1 = nn.Sequential(
            nn.Linear(in_channels, self.hidden_size*2),
            nn.Dropout(0.5),
            nn.ReLU(),
            nn.Linear(self.hidden_size*2, self.hidden_size*2),
            nn.Dropout(0.5),
            nn.ReLU(),
            nn.Linear(self.hidden_size*2, self.hidden_size*2),
            nn.Dropout(0.5),
            nn.ReLU(),
        )
        self.lstm = nn.LSTM(self.hidden_size*2, self.hidden_size, batch_first=True)
        self.o1 = nn.Sequential(
            nn.Linear(self.hidden_size, self.output_size),
        )

        self.lstm2 = nn.LSTM(self.hidden_size, self.hidden_size, batch_first=True)
        
        self.final = nn.Sequential(
            nn.Linear(self.hidden_size*2, self.output_size)
        )
        

    def init_hidden(self, minibatch_size):
        # Before we've done anything, we dont have any hidden state.
        # Refer to the Pytorch documentation to see exactly
        # why they have this dimensionality.
        # The axes semantics are (num_layers, minibatch_size, hidden_dim)
        if self.iscuda:
            return (Variable(torch.zeros(1, minibatch_size, self.hidden_size).cuda()),
                    Variable(torch.zeros(1, minibatch_size, self.hidden_size).cuda()))
        else:
            return (Variable(torch.zeros(1, minibatch_size, self.hidden_size)),
                    Variable(torch.zeros(1, minibatch_size, self.hidden_size)))        

        
    def forward(self, input):
        self.x1 = self.i1(input.transpose(1,2))
        batch_size = input.size(0)
        if self.stage == 0:
            output = self.final(self.x1).view(batch_size, input.size(2), self.output_size)
        elif self.stage == 1:
            self.hidden = self.init_hidden(batch_size)
            
            self.x2, self.hidden = self.lstm(self.x1, self.hidden)
            output = self.o1(self.x2).view(batch_size, input.size(2), self.output_size)
        else:
            self.hidden1 = self.init_hidden(batch_size)
            self.hidden2 = self.init_hidden(batch_size)
            self.x2, self.hidden1 = self.lstm(self.x1, self.hidden1)
            self.x3, self.hidden2 = self.lstm2(self.x2, self.hidden1)
            output = self.o1(self.x3).view(batch_size, input.size(2), self.output_size)
            
        if self.one_hot:
            output = output.transpose(1,2)
        return output
        

    def change_net(self,stage):
        self.stage = stage
        if stage == 1:
            for param in self.i1.parameters():
                param.requires_grad = False

    def grad_clip(self, cutoff):
        #print torch.sum(self.x1.grad)
        #calc_gradients(self.lstm.parameters())
        torch.nn.utils.clip_grad_norm_(self.lstm.parameters(), cutoff)
