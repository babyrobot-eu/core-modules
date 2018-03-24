import os
import pickle
from collections import Counter

import numpy
from babyrobot.emorec_pytorch.config import General
from babyrobot.emorec_pytorch.model.read_data import get_emotion_data
from babyrobot.emorec_pytorch.model.utilities import index_array
from sklearn.model_selection import StratifiedShuffleSplit
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import LabelEncoder, MinMaxScaler, StandardScaler
from torch.utils.data import Dataset


class DataManager:

    def __init__(self, transform=None, simplify=False):
        """

        Args:
            transform (list): a list of callable that apply transformation
                on the samples.
        """

        # import external configuration settings.
        # It doesn't feel like a good practice, maybe change it...
        self.config = General()

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

        self.data, self.target = self.prepare_dataset(data, simplify)

        # create mapping for the categorical labels
        self.label_cat_encoder = LabelEncoder()
        self.label_cat_encoder.fit(self.target[1])
        # scale the continuous labels
        self.label_cont_encoder = MinMaxScaler()
        self.label_cont_encoder.fit(self.target[0])

        # standardize the data
        self.normalizer = Pipeline([
            # ('normalizer', MinMaxScaler(feature_range=(-1, 1))),
            ('standardizer', StandardScaler()),
            # ('normalizer', Normalizer(norm='l2', copy=False)),
        ])
        # self.normanizer = StandardScaler()
        self.normalizer.fit([s for u in self.data for s in u])

        self.max_length = max([len(x) for x in self.data])

        self.input_size = self.data[0].shape[1]

        # plt.hist(lenghts, normed=True, bins=20, range=(0, self.max_length))
        # plt.ylabel('utterances')
        # plt.xlabel('number of segments')
        # plt.show()

    def prepare_dataset(self, data, simplify):
        X = [x[1] for x in data]

        # continuous labels
        y1 = [[x[2][k] for k in ['valence', 'arousal', 'dominance']]
              for x in data]

        # categorical labels
        y2 = [x[2]["emotion"] for x in data]

        if simplify:
            X, y1, y2 = self.simplify_labels(X, y1, y2)

        return X, [y1, y2]

    def simplify_labels(self, data, labels_cont, labels_cat):
        counts = Counter(labels_cat)
        print('categorical labels (initial):')
        print(counts)
        # plot_dict(counts)

        # filter labels
        mask = [x not in self.config.omit_labels for x in labels_cat]
        labels_cont = numpy.array(labels_cont)[mask]
        labels_cat = numpy.array(labels_cat)[mask]
        data = numpy.array(data)[mask]

        # apply mapping
        labels_cat = [self.config.label_map[x] for x in labels_cat]

        counts = Counter(labels_cat)
        print('categorical labels (simplified):')
        print(counts)
        # plot_dict(counts)

        return data, labels_cont, labels_cat

    def pad_sample(self, sample):
        z_pad = numpy.zeros((self.max_length, sample.shape[1]))
        for i in range(min(self.max_length, sample.shape[0] )):
            z_pad[i] = sample[i]
        return z_pad

    def get_split(self, indices):
        """
        Select a subset of the dataset with the given indices
        Args:
            indices (): array of indices

        Returns:

        """
        X = index_array(self.data, indices)
        y_cont = index_array(self.target[0], indices)
        y_cat = index_array(self.target[1], indices)
        return X, [y_cont, y_cat]

    def split(self, ratio=0.2):
        """
        Get a stratified split of the dataset
        Args:
            ratio ():

        Returns:

        """
        sss = StratifiedShuffleSplit(n_splits=1,
                                     test_size=ratio,
                                     random_state=0)
        train_indices, test_indices = list(sss.split(self.data,
                                                     self.target[1]))[0]

        X_train, y_train = self.get_split(train_indices)
        X_test, y_test = self.get_split(test_indices)

        return (X_train, y_train), (X_test, y_test)

    def prep_sample(self, sample):
        _sample = sample
        for i, tsfrm in enumerate(self.transform):
            _sample = tsfrm(_sample)

        # standardize the data
        _sample = self.normalizer.transform(_sample).astype('float32')
        # zero padding, up to self.max_length
        _sample = self.pad_sample(_sample)

        return _sample, len(sample)

    def __len__(self):
        return len(self.data)


class EmotionDataset(Dataset):

    def __init__(self, data, targets, data_manager, transform=None):
        """

        Args:
            transform (list): a list of callable that apply transformation
                on the samples.
        """

        if transform is None:
            transform = []
        self.transform = transform

        self.data = data
        self.targets = targets
        self.data_mngr = data_manager

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
        label = self.targets[0][index], self.targets[1][index]

        for i, tsfrm in enumerate(self.transform):
            sample = tsfrm(sample)

        # standardize the data
        sample = self.data_mngr.normalizer.transform(sample).astype('float32')
        # zero padding, up to self.max_length
        sample = self.data_mngr.pad_sample(sample).astype('float32')

        # convert string categorical labels, to class ids
        cat_label = self.data_mngr.label_cat_encoder.transform([label[1]])[0]
        # convert continuous labels, to desired range (0-1)
        cont_label = self.data_mngr.label_cont_encoder.transform(
            [label[0]]).ravel().astype('float32')

        return sample, cont_label, cat_label, len(self.data[index]), index
