import os
import pickle
from collections import Counter

import numpy
from emorec.model.src.read_data import get_emotion_data
from sklearn.preprocessing import LabelEncoder, MinMaxScaler, StandardScaler
from torch.utils.data import Dataset


class EmotionDataset(Dataset):

    def __init__(self, transform=None):
        """

        Args:
            transform (list): a list of callable that apply transformation
                on the samples.
        """

        if transform is None:
            transform = []
        self.transform = transform

        _dir = os.path.dirname(os.path.abspath(__file__))
        self._cached_file = os.path.join(_dir, "dataset", "_emo_data.pickle")

        if os.path.exists(self._cached_file):
            print("Loading dataset from cache file...",)
            with open(self._cached_file, 'rb') as f:
                data = pickle.load(f)
            print("done!")
        else:
            with open(self._cached_file, "wb") as f:
                print("loading dataset...")
                data = list(get_emotion_data())
                print("caching dataset...")
                pickle.dump(data, f)

        self.data, self.target = self.prepare_data(data)

        # create mapping for the categorical data
        self.label_cat_encoder = LabelEncoder()
        self.label_cat_encoder.fit(self.target[1])
        self.label_cont_encoder = MinMaxScaler()
        self.label_cont_encoder.fit(self.target[0])

        # standardize the data
        self.scaler = StandardScaler()
        self.scaler.fit([s for u in self.data for s in u])

        lenghts = [len(x) for x in self.data]
        self.max_length = max(lenghts)

        self.input_size = self.data[0].shape[1]

        # plt.hist(lenghts, normed=True, bins=20, range=(0, self.max_length))
        # plt.ylabel('utterances')
        # plt.xlabel('number of segments')
        # plt.show()

    def prepare_data(self, data):
        X = [x[1] for x in data]

        # continuous labels
        y1 = [[x[2][k] for k in ['valence', 'arousal', 'dominance']]
              for x in data]

        # categorical labels
        y2 = [x[2]["emotion"] for x in data]
        X, y1, y2 = self.simplify(X, y1, y2)

        return X, [y1, y2]

    def simplify(self, data, labels_cont, labels_cat):
        counts = Counter(labels_cat)
        print('categorical labels (initial):')
        print(counts)
        # plot_dict(counts)

        _map = {
            'frustrated': "negative",
            'neutral': "neutral",
            'angry': "negative",
            'sad': "negative",
            'excited': "positive",
            'happy': "positive",
            'surprised': "sad",
            'fearful': "negative",
            'disgusted': "negative"
        }
        _omit = {"surprised"}

        # filter labels
        mask = [x not in _omit for x in labels_cat]
        labels_cont = numpy.array(labels_cont)[mask]
        labels_cat = numpy.array(labels_cat)[mask]
        data = numpy.array(data)[mask]

        # apply mapping
        labels_cat = [_map[x] for x in labels_cat]

        counts = Counter(labels_cat)
        print('categorical labels (simplified):')
        print(counts)
        # plot_dict(counts)

        return data, labels_cont, labels_cat

    def pad_sample(self, sample):
        z_pad = numpy.zeros((self.max_length, sample.shape[1]))
        z_pad[:sample.shape[0], :sample.shape[1]] = sample
        return z_pad

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        """
        Returns the _transformed_ item from the dataset

        Args:
            index (int):

        Returns:
            (tuple):
            * example (ndarray): vector representation of a training example
            * label (string): the class label
            * length (int): the length (segments) of the utterance
            * index (int): the index of the returned dataitem in the dataset.
              It is useful for getting the raw input for visualizations.

        """
        sample = self.data[index]
        label = self.target[0][index], self.target[1][index]

        for i, tsfrm in enumerate(self.transform):
            sample = tsfrm(sample)

        # standardize the data
        sample = self.scaler.transform(sample).astype('float32')
        # zero padding, up to self.max_length
        sample = self.pad_sample(sample)

        # convert string categorical labels, to class ids
        cat_label = self.label_cat_encoder.transform([label[1]])[0]
        # convert continuous labels, to desired range (0-1)
        cont_label = self.label_cont_encoder.transform(
            [label[0]]).ravel().astype('float32')
        label = cont_label, cat_label

        return sample, label, len(self.data[index]), index

# ee = EmotionDataset()
# ee[1]
