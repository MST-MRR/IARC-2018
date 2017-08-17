############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Utility class module for loading h5py datasets and splitting them into stratified train/test sets.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import atexit

import h5py
import numpy as np

from sklearn.model_selection import StratifiedKFold

from .data import DATASET_LABEL, RANDOM_SEED
from .util import TempH5pyFile

# default number of stratified folds to split each dataset into
DEFAULT_NUM_FOLDS = 3
# default percentage of the dataset to set aside for training
DEFAULT_PERCENT_TRAIN = .8

# raised when ClassifierDataset is instantiated without using a context manager
CONTEXT_MANAGER_NOT_USED_ERROR = 'You must instantiate the object with a context manager before calling this function'

class ClassifierDataset():
    """
    Utility class that aids with loading h5py datasets and splitting them into stratified train/test sets.

    Parameters
    ----------
    dataset_file_path: str
        Path to the positive dataset file, or in the case of multiclass classification datasets, the path to the entire dataset.
    neg_dataset_file_path: str, optional
        Path to the negative dataset file, required only for binary classifiers.
    labels: numpy.ndarray, optional
        Class labels for the dataset, required for multiclass classifcation datasets.

    Notes
    ----------
    For binary classification problems, the labels are generated assuming 1 represents the positive class and 0 represents
    the negative class.

    Examples
    --------
    >>> with ClassifierDataset('pos.hdf', 'neg.hdf', None) as dataset:
    ...    X_train, X_test, y_train, y_test = dataset.get_stratified_training_set()
    ...    # do something with the datasets

    In the above example, it is assumed that you are splitting a dataset for a binary classifier. As shown in the example above, you
    don't have to provide class labels, since they will be generated correctly for you as long as pos.hdf only contains positive samples
    and neg.hdf only contains negative samples.
    """

    def __init__(self, dataset_file_path, neg_dataset_file_path=None, labels=None):
        """
        Saves initialization parameters and initializes internal attributes.
        """

        self.dataset_file_path = dataset_file_path
        self.neg_dataset_file_path = neg_dataset_file_path
        self.labels = labels
        self.ref_count = 0
        self.dataset = None
        self.dataset_file = None
        self.negatives_file = None
        self.negatives = None


    def __enter__(self):
        """
        Reads in all relevant h5py datasets and combines them into one training set when this object is instantiated by a context manager.
        """

        self.ref_count += 1

        dataset_len = 0
        use_negatives = self.neg_dataset_file_path is not None

        self.dataset_file = h5py.File(self.dataset_file_path, 'r')
        self.dataset = self.dataset_file[DATASET_LABEL]
        dataset_len += len(self.dataset)

        if use_negatives:
            self.negatives_file = h5py.File(self.neg_dataset_file_path, 'r')
            self.negatives = self.negatives_file[DATASET_LABEL]
            dataset_len += len(self.negatives)
        else:
            self.negatives = None

        self.training_set = TempH5pyFile('a').__enter__()
        self.training_set.create_dataset(DATASET_LABEL, (dataset_len,) + (self.dataset.shape[1:]), dtype = self.dataset.dtype)
        self.training_set[DATASET_LABEL][:len(self.dataset)] = self.dataset

        if use_negatives:
            self.training_set[DATASET_LABEL][len(self.dataset):] = self.negatives

        atexit.register(self.__exit__)
        return self

    def __exit__(self, *args):
        """
        Attempts to free all dataset resources when the end of the context manager is reached or an exception is thrown.
        """

        if self.ref_count > 0:
            for database_file in (self.dataset, self.negatives, self.training_set):
                try:
                    database_file.close()
                except:
                    pass

        self.ref_count -= 1

    def assert_using_context_manager(self):
        """
        Assert that the caller is using a context manager with this object.

        Raises
        ------
        AssertionError
            If there was an attempt to use this object without a context manager.
        
        Returns
        -------
        None
        """

        assert self.ref_count == 1, CONTEXT_MANAGER_NOT_USED_ERROR

    def stratified_splitter(self, folds=DEFAULT_NUM_FOLDS, seed=RANDOM_SEED):
        """
        Generates folds of stratified train/test sets for model cross-validation

        Parameters
        ----------
        folds: int, optional
            Number of folds to use, default is `DEFAULT_NUM_FOLDS`.
        seed: int, optional
            Random number generator seed, default is `RANDOM_SEED`.

        Returns
        -------
        out: generator
        Returns a generator which yields train samples, test samples, train class labels, and test class labels for each fold.
        """

        self.assert_using_context_manager()

        skfolds = StratifiedKFold(n_splits=folds, random_state=seed)
        X = self.training_set[DATASET_LABEL]
        y = np.vstack((np.ones((len(self.dataset), 1)), np.zeros((len(self.negatives), 1)))) if self.labels is None else self.labels

        for train_index, test_index in skfolds.split(np.zeros(len(y)), y.ravel()):
            X_train, X_test, y_train, y_test = (X[list(train_index)], X[list(test_index)], y[train_index], y[test_index])
            yield X_train, X_test, y_train, y_test

    def get_stratified_training_set(self, percent_train=DEFAULT_PERCENT_TRAIN, seed=RANDOM_SEED):
        """
        Get a stratified train-test split.

        Parameters
        ----------
        percent_train: float, optional
            Percentage of the dataset to set aside for training, default is `DEFAULT_PERCENT_TRAIN`.
        seed: int, optional
            Random number generator seed, default is `RANDOM_SEED`.

        Returns
        -------
        out: numpy.ndarray, numpy.ndarray, numpy.ndarray, numpy.ndarray
        Returns the train samples, test samples, train class labels, and test class labels as a set of numpy arrays.
        """

        num_folds = 1./(1-percent_train)
        X_train, X_test, y_train, y_test = next(self.stratified_splitter(int(num_folds), seed))
        return X_train, X_test, y_train, y_test
