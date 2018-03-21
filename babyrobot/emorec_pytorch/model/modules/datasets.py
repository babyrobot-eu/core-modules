import os
import pickle

from torch.utils.data import Dataset

from babyrobot.lib.config import BASE_PATH


class BaseDataset(Dataset):
    """
    This is a Base class which extends pytorch's Dataset, in order to avoid
    boilerplate code and equip our datasets with functionalities such as
    caching.
    """

    def __init__(self, X, y, max_length=0, name=None):
        """

        Args:
            X (): List of training samples
            y (): List of training labels
            name (str): the name of the dataset. It is needed for caching.
                if None then caching is disabled.
            max_length (int): the max length for each sentence.
                if 0 then use the maximum length in the dataset
        """
        self.data = X
        self.labels = y
        self.name = name
        self.data = self.load_preprocessed_data()
        self.label_transformer = None

        # if max_length == 0, then set max_length
        # to the maximum sentence length in the dataset
        if max_length == 0:
            self.max_length = max([max(len(pair[0]), len(pair[1]))
                                   for pair in self.data])
        else:
            self.max_length = max_length

    def preprocess(self, name, X):
        """
        Preprocessing pipeline
        Args:
            X (list): list of training examples

        Returns: list of processed examples

        """
        raise NotImplementedError

    @staticmethod
    def _check_cache():
        cache_dir = os.path.join(BASE_PATH, "_cache")
        if not os.path.exists(cache_dir):
            os.makedirs(cache_dir)

    def _get_cache_filename(self):
        return os.path.join(BASE_PATH, "_cache",
                            "preprocessed_{}.p".format(self.name))

    def _write_cache(self, data):
        self._check_cache()

        cache_file = self._get_cache_filename()

        with open(cache_file, 'wb') as pickle_file:
            pickle.dump(data, pickle_file)

    def load_preprocessed_data(self):

        # NOT using cache
        if self.name is None:
            print("cache deactivated!")
            return self.preprocess(self.name, self.data)

        # using cache
        cache_file = self._get_cache_filename()

        if os.path.exists(cache_file):
            print("Loading {} from cache!".format(self.name))
            with open(cache_file, 'rb') as f:
                return pickle.load(f)
        else:
            print("No cache file for {} ...".format(self.name))
            data = self.preprocess(self.name, self.data)
            self._write_cache(data)
            return data
