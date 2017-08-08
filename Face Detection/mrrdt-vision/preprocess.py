############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Utility class for normalizing images so that they can be used an inputs to machine learning models.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import h5py
import numpy as np

from keras.preprocessing.image import ImageDataGenerator

from .data import DATASET_LABEL, RANDOM_SEED
from .main import DEBUG

# maximum number of images to sample from each image set
DEFAULT_FIT_SAMPLE_SIZE = 100000
# name for flip keyword argument
FLIP_PARAM_KEY = 'flip'

class ImageNormalizer():
    """
    Lightweight wrapper for keras' image preprocessing functionality

    Parameters
    ----------
    pos_dataset_file_path: str
        Path to the model's positive dataset file.
    neg_dataset_file_path: str
        Path to the model's negative dataset file.
    norm_method: str, optional
        Normalization method to preprocess images with, defaults to `ImageNormalizer.STANDARD_NORMALIZATION`.
    sample_size: int, optional
        Maximum number of images to sample from each image set, defaults to `preprocess.DEFAULT_FIT_SAMPLE_SIZE`.
    """

    # flag for using standard normalization as the normalization method
    STANDARD_NORMALIZATION = 'std_norm'
    # flag for using min max scaling as the normalization method
    MIN_MAX_SCALING = 'min_max'
    # flag for using zca whitening as the normalization method
    ZCA_WHITENING = 'zca'

    # array of the normalization methods supported
    NORM_METHODS = (STANDARD_NORMALIZATION, MIN_MAX_SCALING, ZCA_WHITENING)

    # augment the data with random horizontal flips
    FLIP_HORIZONTAL = 'fliph'
    # augment the data with random vertical flips
    FLIP_VERTICAL = 'flipv'
    # agument the data with random horizontal and vertical flips
    FLIP_HORIZONTAL_AND_VERTICAL = 'flipvh'

    def __init__(self, pos_dataset_file_path, neg_dataset_file_path, norm_method=STANDARD_NORMALIZATION, sample_size=DEFAULT_FIT_SAMPLE_SIZE):
        """
        Cache a dataset to fit the normalizer with (if necessary) and initialize internal attributes.
        """

        self.X = None
        self.preprocess_params = {}
        self.image_preprocessor = None
        self.no_augmentation_image_preprocessor = None
        self.norm_method = norm_method
        self.normalization_params = {}

        if self._is_fit_necessary():
          for file_path in (pos_dataset_file_path, neg_dataset_file_path):
              with h5py.File(file_path, 'r') as in_file:
                  dataset = in_file[DATASET_LABEL]

                  if self.X is None:
                      self.X = np.zeros((0,) + (dataset.shape[1:]), dtype=dataset[0].dtype)

                  self.X = np.vstack((self.X, dataset[:min(sample_size, len(dataset))]))
        
        if norm_method == ImageNormalizer.STANDARD_NORMALIZATION:
            self.preprocess_params['featurewise_center'] = True
            self.preprocess_params['featurewise_std_normalization'] = True
        elif norm_method == ImageNormalizer.MIN_MAX_SCALING:
            self.preprocess_params['rescale'] = 1./255
        elif norm_method == ImageNormalizer.ZCA_WHITENING:
            self.preprocess_params['zca_whitening'] = True

        self.normalization_params.update(self.preprocess_params)
        
        self.image_preprocessor = ImageDataGenerator(**self.preprocess_params)
        self.no_augmentation_image_preprocessor = ImageDataGenerator(**self.normalization_params)

        if self._is_fit_necessary():
            self.image_preprocessor.fit(self.X)
            self.no_augmentation_image_preprocessor.fit(self.X)

            del self.X

    def _is_fit_necessary(self):
        """
        Returns whether or the normalizer must be fitted on a dataset before use.

        Returns
        -------
        out: bool
        Returns True if normalizer's fit method must be called before use, false otherwise.
        """
        
        return self.norm_method in (ImageNormalizer.STANDARD_NORMALIZATION, ImageNormalizer.ZCA_WHITENING)

    def add_data_augmentation_params(self, params):
        """
        Add data augmentation steps to the normalizer.

        Parameters
        ----------
        params: dict
            Dictionary of data augmentation parameter name/value pairs.

        Notes
        ----------
        See https://keras.io/preprocessing/image/ for a list of the data augmentation parameters available.

        Returns
        -------
        None
        """

        if FLIP_PARAM_KEY in params.keys():
            flip_val = params.get(FLIP_PARAM_KEY)
            params['vertical_flip'] = flip_val == ImageNormalizer.FLIP_VERTICAL or flip_val == ImageNormalizer.FLIP_HORIZONTAL_AND_VERTICAL
            params['horizontal_flip'] = flip_val == ImageNormalizer.FLIP_HORIZONTAL or flip_val == ImageNormalizer.FLIP_HORIZONTAL_AND_VERTICAL
            del params[FLIP_PARAM_KEY]

        self.preprocess_params.update(params)

    def preprocess(self, images, labels=None, batch_size=None, shuffle=False, use_data_augmentation=True, seed=RANDOM_SEED):
        """
        Trains a model on a dataset for `num_epochs` epochs

        Parameters
        ----------
        images: numpy.ndarray
            Array of images to preprocess.
        labels: numpy.ndarray, optional
            Class labels for the dataset, required only if you have the correct labels for a set of images and you want to keep track
            of which preprocessed sample belongs to which class.
        batch_size: int, optional
            Minibatch size, if not used, this function preprocess every image in `images` and returns the resulting array of preprocesed images.
        shuffle: bool, optional
            Whether or not to shuffle the preprocessed images into a pseudo-random order, default is False.
        use_data_augmentation: bool, optional
            Whether or not to make use of any set data augmentation parameters, default is True.
        seed: int, optional
            Random number generator seed, default is `RANDOM_SEED`.
          
        
        Returns
        -------
        out: generator or numpy.ndarray
        Returns a generator that yields preprocessed samples in an infinite loop if `batch_size` was non-None, returns a preprocessed
        version of the `images` param otherwise. 
        """

        if len(images) == 0:
            return images
        
        return_generator = batch_size is not None
        preprocessor = self.image_preprocessor if use_data_augmentation else self.no_augmentation_image_preprocessor
        labels = np.zeros(len(images)) if labels is None else labels
        batch_size = len(images) if batch_size is None else batch_size

        generator = preprocessor.flow(images, labels, batch_size=batch_size, shuffle=shuffle, seed=seed)
        return generator if return_generator else next(generator)[0]
