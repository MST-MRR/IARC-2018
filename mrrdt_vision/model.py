############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Collection of classes which together form a convolutional neural network cascade.

Cascade implementation is based primarily on [1], but some techniques used have been adopted from various sources
on the web.

Training speed improvements and some hyperparameter selections were inspired by [2]

References
----------
.. [1] Li, H., Lin, Z., Shen, X., Brandt, J., & Hua, G. (2015). A convolutional neural network cascade for face detection. 
       In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (pp. 5325-5334).

.. [2] Goyal, P., Dollár, P., Girshick, R., Noordhuis, P., Wesolowski, L., Kyrola, A., & He, K. (2017). 
       Accurate, Large Minibatch SGD: Training ImageNet in 1 Hour. arXiv preprint arXiv:1706.02677.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import pickle
import os
import cv2
import numpy as np
import atexit
import copy
import time
import six
import gc
import abc
from abc import abstractmethod
from collections import namedtuple, Iterable

import keras
from keras.models import Sequential, Model, load_model
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, BatchNormalization, Input, concatenate
from keras.engine.topology import InputLayer
from keras.optimizers import SGD
from keras.utils import np_utils
from keras.callbacks import ModelCheckpoint
from keras import backend as K

from .hyperopt_keras import HyperoptWrapper, get_best_params, DEFAULT_NUM_EVALS, tune
from .preprocess import ImageNormalizer
from .data import SCALES, DATASET_LABEL, DatasetManager
from .util import TempH5pyFile
hp = HyperoptWrapper()

# set to true when testing experimental code.
DEBUG = False

# file names for every network in the cascade. You may access a particular network name by indexing first by whether
#   that network is a calibration net or not, and then by the image scale that that network accepts as a primary input
NET_FILE_NAMES = {False: {SCALES[0][0]: '12net.hdf', SCALES[1][0]: '24net.hdf', SCALES[2][0]: '48net.hdf'}, 
                  True: {SCALES[0][0]: '12calibnet.hdf', SCALES[1][0]: '24calibnet.hdf', SCALES[2][0]: '48calibnet.hdf'}}

# dropout parameter names will start with this identifier and end with a unique index number
DROPOUT_PARAM_ID = 'dropout'
# params passed to the model optimizer during compilation
OPTIMIZER_PARAMS = ['lr', 'momentum', 'decay', 'nesterov']
# params used for normalizattion and data augmentation
NORMALIZATION_PARAMS = ['norm', 'flip', 'width_shift_range', 'height_shift_range', 'rotation_range']
# params used during training
TRAIN_PARAMS = ['batchSize', 'batch_size']

# optimizer used during training
OPTIMIZER = SGD

# default training minibatch size
DEFAULT_BATCH_SIZE = 128
# default prediction batch size
PREDICTION_BATCH_SIZE = 256
# default number of training epochs
DEFAULT_NUM_EPOCHS = 300
# default queue size for cached fit_generator batches
DEFAULT_Q_SIZE =  32
# weights file to use when debugging training code
DEBUG_FILE_PATH = 'debug.hdf'
# param file name template
PARAM_FILE_NAME_FORMAT_STR = '%sparams'

# error message displayed when an attempt is made to train stage two before stage one
STAGE_ONE_NOT_TRAINED_ERROR = 'You must train the stage one models before moving onto stage two!'
# error message displayed when an attempt is made to train stage three before stage two
STAGE_TWO_NOT_TRAINED_ERROR = 'You must train the stage two models before moving onto stage three!'

def _convert(data):
    """
    Converts data to a format readable by both Python 2 and Python 3

    Parameters
    ----------
    data: object
        Data to decode.

    Returns
    -------
    out: object
    Python 2 and 3 friendly version of `data`
    """

    if isinstance(data, bytes):
        return data.decode('ascii')
    elif isinstance(data, (dict, tuple, list, set)):
        return type(data)(map(_convert, data.items() if hasattr(data, 'items') else data))

    return data

@six.add_metaclass(abc.ABCMeta)
class ObjectClassifier():
    """
    Abstract base class for models that classify an image as object or non-object

    Parameters
    ----------
    stage_idx: int
        Cascade stage index
    
    Usage
    ----------
    This base class will handle training, prediction, hyperparameter tuning, and model evaluation. The only required subclass methods are 
    an `__init__` method which calls this class' constructor and a `__call__` method which builds and compiles a keras model.

    Notes
    ----------
    You cannot instantiate this class directly.

    See Also
    ----------
    `model.StageOneClassifier`

    `model.StageTwoClassifier`

    `model.StageThreeClassifier`
    """

    # default dropout layer percentage value
    DEFAULT_DROPOUT = .3
    # loss function used for binary classification
    LOSS = 'binary_crossentropy'
    # keras metrics used besides loss
    METRICS = ['accuracy']

    @abstractmethod
    def __init__(self, stage_idx):
        """
        Sets up the classifier's internal attributes and loads in saved model parameters when available.
        """

        self.img_size = SCALES[stage_idx]
        self.input_shape = (self.img_size[1], self.img_size[0], 3)
        self.additional_normalizers = []
        self.trained_model = None
        self.best_params = None
        self.base_folder = ''

        if stage_idx == 2:
            self.PARAM_SPACE = copy.deepcopy(self.PARAM_SPACE)
            self.PARAM_SPACE.update({'dropout2': self.hp.uniform(0, .75)})

        self.update()

    @abstractmethod
    def __call__(self):
        """
        Abstract method that must be implemented by any subclass of this class.

        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled..
        """

        pass

    def get_stage_idx(self):
        """
        Get the model's cascade stage index.

        Returns
        -------
        out: int
        Returns the model's cascade stage index.
        """

        return self.stage_idx
    
    def get_param_file_path(self):
        """
        Get the model's parameter file path.

        Returns
        -------
        out: str
        Returns the model's parameter file path.
        """

        param_file_path = PARAM_FILE_NAME_FORMAT_STR % (os.path.splitext(self.get_weights_file_path())[0],)
        return param_file_path

    def get_param_space(self):
        """
        Get the model's hyperparameter space.

        Returns
        -------
        out: dict
        Returns the model's hyperparameter space.
        """

        return self.PARAM_SPACE

    def get_weights_file_path(self):
        """
        Get the model's weights file path.

        Returns
        -------
        out: str
        Returns the model's weights file path.
        """

        weights_file_path = NET_FILE_NAMES[isinstance(self, ObjectCalibrator)][SCALES[self.stage_idx][0]]
        return os.path.join(os.path.dirname(os.path.realpath(__file__)), self.base_folder, weights_file_path)

    def get_normalization_method(self):
        """
        Get the model's input normalization method.

        Returns
        -------
        out: str
        Returns the model's input normalization method.
        """

        return self.best_params['norm'] if self.was_tuned() else ImageNormalizer.STANDARD_NORMALIZATION

    def get_normalization_params(self):
        """
        Get the model's parameters for normalization.

        Returns
        -------
        out: dict
        Returns the model's parameters for normalization as a dictionary of name-value pairs.
        """

        params = {}

        if self.was_tuned():
            params = {k: v for k, v in self.best_params.items() if k in NORMALIZATION_PARAMS}
            del params['norm']

        return params

    def get_additional_normalizers(self, dataset_manager_params=None):
        """
        Get the model's additional normalizers if there are any.

        Parameters
        ----------
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to None.
            Required if `self.additional_normalizers` is empty.

        Returns
        -------
        out: list
        Returns the model's additional normalizers if there are any.
        """

        if not self.additional_normalizers:
            for i in np.arange(0, self.stage_idx):
                self.additional_normalizers.append(DatasetManager(MODELS[i][0], **dataset_manager_params).get_normalizer())

        return self.additional_normalizers

    def get_dropouts(self):
        """
        Get the model's dropout percentage values.

        Returns
        -------
        out: list
        Returns a list of the model's dropout percentage values.
        """

        dropouts = [self.DEFAULT_DROPOUT] * len([k for k in self.get_param_space() if k.startswith(DROPOUT_PARAM_ID)])

        if self.was_tuned():
            dropouts = []
            for k, v in self.best_params.items():
                if k.startswith(DROPOUT_PARAM_ID):
                    idx = int(k.replace(DROPOUT_PARAM_ID, ''))
                    dropouts.insert(idx, v)
                    
        return dropouts

    def get_optimizer_params(self):
        """
        Get the model's optimizer parameters as a dictionary of name-value pairs.

        Returns
        -------
        out: dict
        Returns the model's optimizer parameters as dictionary of name-value pairs.
        """

        return {k: v for k, v in self.best_params.items() if k in OPTIMIZER_PARAMS} if self.was_tuned() else {}

    def get_batch_size(self):
        """
        Get the model's minibatch size

        Returns
        -------
        out: int
        Returns the model's minibatch size
        """

        # batchSize key is for backwards compatibility
        return DEFAULT_BATCH_SIZE if not self.was_tuned() else (self.best_params.get('batchSize') or self.best_params.get('batch_size'))

    def get_save_file_path(self, debug=DEBUG):
        """
        Get the model's save file path.

        Parameters
        ----------
        debug: bool, optional
            Controls whether or not to return the debug mode's save file path, default is `main.DEBUG`.
        
        Returns
        -------
        out: str
        Returns the model's save file path
        """

        return self.get_weights_file_path() if not debug else DEBUG_FILE_PATH

    def set_base_folder(self, base_folder):
        """
        Set the folder that this model's files are based in.

        Parameters
        ----------
        base_folder: str
            Path to the folder to base the model's files in.
        
        Returns
        -------
        None
        """

        self.base_folder = base_folder
        self.update()

    def was_tuned(self):
        """
        Get whether or not the model's hyperparameters were tuned.

        Returns
        -------
        out: bool
        Returns True if tuned model hyperparameters are available, False otherwise.
        """
        return os.path.isfile(self.get_param_file_path())

    def update(self):
        """
        Check for and load in tuned hyperparameters for this model when available.

        Returns
        -------
        None
        """

        if self.was_tuned():
            with open(self.get_param_file_path(), 'rb') as param_file:
                loadArgs = {} if six.PY2 else {'encoding': 'bytes'}
                trials = pickle.load(param_file, **loadArgs)
                trials.__dict__  = _convert(trials.__dict__)
                best = _convert(trials.best_trial['misc']['vals'])
                self.best_params = get_best_params(self.get_param_space(), best)

    def compile(self, params = {}, loss = LOSS, metrics = METRICS):
        """
        Compile self.model with the given parameters

        Parameters
        ----------
        params: dict, optional
            Dictionary for the optimizer's hyperparameters.
        loss: str, optional
            Keras loss function, defaults to `ObjectClassifier.LOSS`.

            See https://keras.io/losses/ for more information.
        metrics: list, optional
            List of Keras metrics to use during training, defaults to 'ObjectClassifier.METRICS'.

            See https://keras.io/metrics/ for more information.


        Returns
        -------
        None
        """

        if len(params) == 0 and self.best_params: params = self.best_params
        self.model.compile(loss=loss, optimizer=OPTIMIZER(**params), metrics=metrics)

    def tune(self, dataset_manager, labels, metric, verbose=True, num_evals=DEFAULT_NUM_EVALS):
        """
        Tune the model's hyperparameters

        Parameters
        ----------
        dataset_manager: data.DatasetManager
            DatasetManager for the given model
        labels: numpy.ndarray
            Class labels for each dataset sample
        metric: callable
            Scikit-learn metric to optimize
        verbose: bool, optional
            Controls whether or not the parameters and results of each trial is shown, default is True.
        num_evals: int, optional
            Number of distinct parameter sets to evaluate, default is `DEFAULT_NUM_EVALS`.

        Returns
        -------
        None
        """

        param_space = self.get_param_space()
        param_file_path = self.get_param_file_path()

        best, trials = tune(param_space, self, dataset_manager, labels, metric, verbose=verbose, num_evals=num_evals)

        if verbose:
            print('Best model parameters found:', get_best_params(param_space, best))

        with open(param_file_path, 'wb') as model_param_file:
            pickle.dump(trials, model_param_file, protocol = 2)

        self.update()

    def get_input_generator(self, X, y, normalizers, **normalizer_params):
        """
        Generate training and validation data for `keras.models.Model.fit_generator`

        Parameters
        ----------
        X: sequence of numpy.ndarray
            Sequence of model inputs
        y: numpy.ndarray
            Array of class labels for each sample in X
        normalizers: list of preprocess.ImageNormalizer
            List of normalizers for each model input
        **normalizer_params: dict
            Keyword parameters to be passed to the input normalizers

        Returns
        -------
        out: generator
        Returns a generator which yields batches of normalized model inputs and their class labels
        """

        is_calibration_net = isinstance(self, ObjectCalibrator)
        normalizer_params.update({'labels': y})
        generator = normalizers[self.stage_idx if not is_calibration_net else 0].preprocess(X, **normalizer_params)
        normalizer_params.update({'batch_size': None, 'labels': None, 'use_data_augmentation': False})

        while True:
                X, y = next(generator)
                X_extended = []
                
                if not is_calibration_net:
                    for i in np.arange(0, self.stage_idx+1):
                        if i != self.stage_idx:
                            resized_inputs = np.vstack([cv2.resize(img, SCALES[i])[np.newaxis] for img in X])
                            X_extended.append(normalizers[i].preprocess(resized_inputs, **normalizer_params))

                X_extended.insert(self.stage_idx, X)

                yield X_extended, y

    def fit(self, X_train, X_test, y_train, y_test, dataset_manager, normalizer=None, num_epochs=DEFAULT_NUM_EPOCHS, 
            save_file_path=None, batch_size=None, optimizer_params=None, dropouts=None, verbose=True, debug=DEBUG):
        """
        Trains a model on a dataset for `num_epochs` epochs

        Parameters
        ----------
        X_train: numpy.ndarray
            Train dataset
        X_test: numpy.ndarray
            Test dataset.
        y_train: numpy.ndarray
            Train dataset's class labels.
        y_test: numpy.ndarray
            Test dataset's class labels.
        dataset_manager: data.DatasetManager
            DatasetManager for the given model.
        normalizer: preprocess.ImageNormalizer, optional
            Train/test data normalizer.
        num_epochs: int, optional
            Number of training epochs, defaults to `DEFAULT_NUM_EPOCHS`.
        save_file_path: str, optional
            File to save model weights to when training, defaults to self.get_save_file_path()
        batch_size: int, optional
            Minibatch size, defaults to the batch size value saved in the model's parameter file or `DEFAULT_BATCH_SIZE` if the model was not tuned.
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        verbose: bool, optional
            Controls whether or not to display training progress, default is True.
        debug: bool, optional
            Controls whether or not to save the model's weights to `DEBUG_FILE_PATH`, default is `main.DEBUG`
        
        Returns
        -------
        None
        """
        
        params = [optimizer_params or self.get_optimizer_params(), dropouts or self.get_dropouts(), dataset_manager.get_params()]
        save_file_path = save_file_path or self.get_save_file_path(debug)
        batch_size = batch_size or self.get_batch_size()

        callbacks = [ModelCheckpoint(save_file_path, monitor='val_loss', save_best_only=True, verbose=int(verbose))]

        model = self(*params)
        y_train, y_test = (np_utils.to_categorical(vec, int(np.amax(vec) + 1)) for vec in (y_train, y_test))

        normalizers = self.additional_normalizers + [normalizer or dataset_manager.get_normalizer()]

        if debug: print(params, save_file_path, batch_size, X_train.shape, X_test.shape, y_train.shape, y_test.shape)

        model.fit_generator(self.get_input_generator(X_train, y_train, normalizers, shuffle=True, batch_size=batch_size),
                            len(X_train)//batch_size,
                            epochs = num_epochs,
                            verbose = 2 if verbose else 0,
                            callbacks = callbacks,
                            validation_data = self.get_input_generator(X_test, y_test, normalizers, batch_size=batch_size, use_data_augmentation=False, shuffle=False),
                            validation_steps = len(X_test)//batch_size,
                            max_queue_size = DEFAULT_Q_SIZE)

        del self.trained_model
        self.trained_model = None
        K.clear_session()
        gc.collect()

    def load_model(self, weights_file_path=None):
        """
        Load a previously trained model from a file

        Parameters
        ----------
        weights_file_path: str, optional
            Path to the model's weights file, defaults to self.get_weights_file_path().

        Returns
        -------
        out: keras.models.Model
        Returns the trained model
        """

        if self.trained_model is None:
            self.trained_model = load_model(self.get_weights_file_path() if weights_file_path is None else weights_file_path)
            inputLayers = []

            for layer in self.trained_model.layers:
                if type(layer) is InputLayer:
                    inputLayers.append(layer.input)

            inputLayers.append(K.learning_phase())
            self.predictFunc = K.function(inputLayers, [self.trained_model.layers[-1].output])
        
        return self.trained_model

    def predict(self, X, normalizer=None, weights_file_path=None, dataset_manager=None):
        """
        Make predictions on an array

        Parameters
        ----------
        X: numpy.ndarray
            Array of data to make predictions on
        normalizer: preprocess.ImageNormalizer, optional
            Normalizer for the data in X, optional if X is already normalized
        weights_file_path: str, optional
            Path to the model's weights file, defaults to self.get_weights_file_path().
        dataset_manager: data.DatasetManager
            DatasetManager for the given model, optional if X is already normalized
        
        Notes
        ----------
        This function operates in two modes. If no normalizer or dataset manager is given,
        the function will operate in "fast" mode by making predictions on X under the assumption 
        that X has already been normalized. Such an assumption is safe since even if X has not been normalized, 
        lower prediction quality will be the only side-effect. If a normalizer and dataset manager are given, the
        function will operate in "slow" mode and handle all of the data preparation needed before making predictions. 
        
        Developers who work on this function in the future may find it useful to add a warning when predictions are
        made on an unnormalized `X`. A trivial way to implement this would be to warn the user when the maximum value of `X` 
        exceeds a set value.

        Returns
        -------
        out: numpy.ndarray
        Returns predictions made by the model
        """

        self.load_model(weights_file_path)
        use_fast_predict = dataset_manager is None
        make_prediction = lambda X: self.predictFunc(X)[0]

        if not use_fast_predict:
            batch_size = PREDICTION_BATCH_SIZE
            normalizers = self.get_additional_normalizers(dataset_manager.get_params()) + [normalizer]
            input_generator = self.get_input_generator(X, None, normalizers, batch_size=batch_size, shuffle=False, use_data_augmentation=False)
            
            for i in np.arange(0, len(X), PREDICTION_BATCH_SIZE):
                X_batch, y_batch = next(input_generator)
                batches = []

                for j, inputArray in enumerate(X_batch):
                    arraySlice = inputArray[:min(PREDICTION_BATCH_SIZE, len(X) - i)]

                    if j < 2:
                        batches.insert(0, arraySlice)
                    else:
                        batches.append(arraySlice)
                
                batches.append(0)
                predictions = make_prediction(batches)
                
                if i == 0: 
                    y = np.zeros((0, predictions.shape[1]))
                
                y = np.vstack((y, predictions))
        else:
            y = make_prediction(X + [0])
        
        return y

    def eval(self, X_test, y_test, normalizer, metric, weights_file_path=None, dataset_manager=None, **metric_kwargs):
        """
        Evaluate the model on a test set with the given metric

        Parameters
        ----------
        X_test: numpy.ndarray
            Test dataset.
        y_test: numpy.ndarray
            Validation dataset's class labels.
        dataset_manager: data.DatasetManager
            DatasetManager for the given model.
        normalizer: preprocess.ImageNormalizer
            Training/validation data normalizer.
        metric: callable
            Scikit-learn metric to evaluate the model with
        weights_file_path: str, optional
            Path to the model's weights file, defaults to self.get_weights_file_path().
        dataset_manager: data.DatasetManager, optional
            DatasetManager for the given model, defaults to None. If dataset_manager is None, fast predict is used.
        **metric_kwargs: dict, optional
            Keyword arguments to pass to the `metric` function.
        
        Returns
        -------
        out: float
        Returns the score returned by the `metric` function
        """

        y_pred = self.predict(X_test, normalizer, weights_file_path, dataset_manager=dataset_manager)
        return metric(y_test, np.argmax(y_pred, axis=1), **metric_kwargs)

@six.add_metaclass(abc.ABCMeta)
class ObjectCalibrator(ObjectClassifier):
    """
    Abstract base class for models that calibrate bounding boxes to better fit target objects

    Parameters
    ----------
    stage_idx: int
        Cascade stage index
    
    Usage
    ----------
    This base class will handle training, prediction, hyperparameter tuning, and model evaluation. The only required subclass methods are 
    an `__init__` method which calls this class' constructor and a `__call__` method which builds and compiles a keras model.

    Notes
    ----------
    You cannot instantiate this class directly.

    See Also
    ----------
    `model.StageOneCalibrator`

    `model.StageTwoCalibrator`

    `model.StageThreeCalibrator`
    """

    LOSS = 'categorical_crossentropy'

    @abstractmethod
    def __init__(self, stage_idx):
        """
        Sets up the calibrator's internal attributes and loads in saved model parameters when available.
        """
        super(ObjectCalibrator, self).__init__(stage_idx)

    @abstractmethod
    def __call__(self):
        """
        Abstract method that must be implemented by any subclass of this class.

        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """
        pass


class StageOneClassifier(ObjectClassifier):
    """
    The classifier for stage one of the cascade.
    """

    HP = HyperoptWrapper()
    PARAM_SPACE = {
        'dropout0': HP.uniform(0, .75),
        'dropout1': HP.uniform(0, .75),
        'lr': HP.loguniform(1e-4, 1),
        'batch_size': HP.choice(512),
        'norm':  HP.choice(ImageNormalizer.STANDARD_NORMALIZATION),
        'flip': HP.choice(ImageNormalizer.FLIP_HORIZONTAL),
        'momentum': HP.choice(.9),
        'decay': HP.choice(1e-4),
        'nesterov': HP.choice(True)
    }

    def __init__(self):
        """
        Sets up the model's internal attributes and loads in saved model hyperparameters when available.
        """

        self.stage_idx = 0
        super(StageOneClassifier, self).__init__(self.stage_idx)
    
    def __call__(self, optimizer_params={}, dropouts=[ObjectClassifier.DEFAULT_DROPOUT]*2, dataset_manager_params={}, include_top=True, compile=True):
        """
        Method used to build and compile the model represented by this class. Overloads the () operator.

        Parameters
        ----------
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to an empty
            dictionary.
        include_top: bool, optional
            Controls whether or not the dense output layer is included in the returned model, default is True.
        compile: bool, optional
            Controls whether or not to compile the returned model, default is True.
        
        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """

        input_layer = Input(shape=self.input_shape)
        conv_2d = Conv2D(16, (3, 3), activation='relu')(input_layer)
        max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(conv_2d)
        first_dropout = Dropout(dropouts[0])(max_pool_2d)

        flattened = Flatten()(first_dropout)
        fully_connected_layer = Dense(16, activation='relu')(flattened)
        final_dropout = Dropout(dropouts[1])(fully_connected_layer)

        if include_top: 
            output_layer = Dense(2, activation='softmax')(final_dropout)
        
        self.model = Model(inputs=input_layer, outputs=output_layer if include_top else fully_connected_layer)

        if compile: self.compile(optimizer_params)

        return self.model

class StageTwoClassifier(ObjectClassifier):
    """
    The classifier for stage two of the cascade.
    """

    HP = HyperoptWrapper()
    PARAM_SPACE = {
        'dropout0': HP.uniform(0, .75),
        'dropout1': HP.uniform(0, .75),
        'lr': HP.loguniform(1e-4, 1),
        'batch_size': HP.choice(512),
        'norm':  HP.choice(ImageNormalizer.STANDARD_NORMALIZATION),
        'flip': HP.choice(ImageNormalizer.FLIP_HORIZONTAL),
        'momentum': HP.choice(.9),
        'decay': HP.choice(1e-4),
        'nesterov': HP.choice(True)
    }

    def __init__(self):
        """
        Sets up the model's internal attributes and loads in saved model hyperparameters when available.
        """

        self.stage_idx = 1
        super(StageTwoClassifier, self).__init__(self.stage_idx)

    def __call__(self, optimizer_params={}, dropouts = [ObjectClassifier.DEFAULT_DROPOUT]*2, dataset_manager_params={}, include_top=True, compile=True):
        """
        Method used to build and compile the model represented by this class. Overloads the () operator.

        Parameters
        ----------
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to an empty
            dictionary.
        include_top: bool, optional
            Controls whether or not the dense output layer is included in the returned model, default is True.
        compile: bool, optional
            Controls whether or not to compile the returned model, default is True.
        

        Raises
        ------
        AssertionError
            If stage one's models were not trained yet.

        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """

        input_layer = Input(shape=self.input_shape)
        conv_2d = Conv2D(64, (5, 5), activation='relu')(input_layer)
        max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(conv_2d)
        first_dropout = Dropout(dropouts[0])(max_pool_2d)

        flattened = Flatten()(first_dropout)
        fully_connected_layer = Dense(128, activation='relu')(flattened)

        stage_one = StageOneClassifier()
        stage_one.set_base_folder(self.base_folder)
        assert os.path.isfile(stage_one.get_weights_file_path()), STAGE_ONE_NOT_TRAINED_ERROR
        stage_one_model = stage_one(dataset_manager_params=dataset_manager_params, include_top=False, compile=False)
        trained_stage_one = stage_one.load_model()

        for i, layer in enumerate(stage_one_model.layers):
            layer.set_weights(trained_stage_one.layers[i].get_weights())
            layer.trainable = False
        
        if not self.additional_normalizers:
            self.additional_normalizers.append(DatasetManager(stage_one, **dataset_manager_params).get_normalizer())
        
        merged_fully_connected_layer = concatenate([fully_connected_layer, stage_one_model.output])
        final_dropout = Dropout(dropouts[1])(merged_fully_connected_layer)

        if include_top:
            output_layer = Dense(2, activation = 'softmax')(final_dropout)

        self.model = Model(inputs=[stage_one_model.input, input_layer], outputs=output_layer if include_top else merged_fully_connected_layer)

        if compile: self.compile(optimizer_params)

        return self.model

class StageThreeClassifier(ObjectClassifier):
    """
    The classifier for stage three of the cascade.
    """

    HP = HyperoptWrapper()
    PARAM_SPACE = {
        'dropout0': HP.uniform(0, .75),
        'dropout1': HP.uniform(0, .75),
        'lr': HP.loguniform(1e-4, 1),
        'batch_size': HP.choice(512),
        'norm':  HP.choice(ImageNormalizer.STANDARD_NORMALIZATION),
        'flip': HP.choice(ImageNormalizer.FLIP_HORIZONTAL),
        'momentum': HP.choice(.9),
        'decay': HP.choice(1e-4),
        'nesterov': HP.choice(True)
    }

    def __init__(self):
        """
        Sets up the model's internal attributes and loads in saved model hyperparameters when available.
        """

        self.stage_idx = 2
        self.hp = StageThreeClassifier.HP
        super(StageThreeClassifier, self).__init__(self.stage_idx)
    
    def __call__(self, optimizer_params={}, dropouts=[ObjectClassifier.DEFAULT_DROPOUT]*3, dataset_manager_params={}):
        """
        Method used to build and compile the model represented by this class. Overloads the () operator.

        Parameters
        ----------
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to an empty
            dictionary.
        
        Raises
        ------
        AssertionError
            If stage two's models were not trained yet.

        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """

        input_layer = Input(shape=self.input_shape)
        conv_2d = Conv2D(64, (5, 5), activation='relu')(input_layer)
        max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(conv_2d)
        first_dropout = Dropout(dropouts[0])(max_pool_2d)
        first_batch_norm = BatchNormalization()(first_dropout)
        secondary_conv_2d = Conv2D(64, (5, 5), activation='relu')(first_batch_norm)
        secondary_batch_norm = BatchNormalization()(secondary_conv_2d)
        secondary_max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(secondary_batch_norm)
        second_dropout = Dropout(dropouts[1])(secondary_max_pool_2d)

        flattened = Flatten()(first_dropout)
        fully_connected_layer = Dense(256, activation='relu')(flattened)

        stage_two = StageTwoClassifier()
        stage_two.set_base_folder(self.base_folder)
        assert os.path.isfile(stage_two.get_weights_file_path()), STAGE_TWO_NOT_TRAINED_ERROR
        trained_stage_two = stage_two.load_model()
        stage_two_model = stage_two(dataset_manager_params=dataset_manager_params, include_top=False, compile=False)
        input_layers = [input_layer]

        for i, layer in enumerate(stage_two_model.layers):
            layer.set_weights(trained_stage_two.layers[i].get_weights())
            layer.trainable = False

            if type(layer) is InputLayer:
                input_layers.insert(0, layer.input)
        
        if not self.additional_normalizers:
            self.additional_normalizers.extend(stage_two.get_additional_normalizers())
            self.additional_normalizers.append(DatasetManager(stage_two, **dataset_manager_params).get_normalizer())

        merged_fully_connected_layer = concatenate([fully_connected_layer, stage_two_model.output])
        third_dropout = Dropout(dropouts[2])(merged_fully_connected_layer)
        output_layer = Dense(2, activation='softmax')(third_dropout)

        self.model = Model(inputs=input_layers, outputs=output_layer)
        self.compile(optimizer_params)
        return self.model

class StageOneCalibrator(ObjectCalibrator):
    """
    The calibrator for stage one of the cascade.
    """

    HP = HyperoptWrapper()
    PARAM_SPACE = {
        'dropout0': HP.uniform(0, .75),
        'dropout1': HP.uniform(0, .75),
        'lr': HP.loguniform(1e-4, 1),
        'batchSize': HP.choice(512),
        'norm':  HP.choice(ImageNormalizer.STANDARD_NORMALIZATION),
        'flip': HP.choice(None),
        'momentum': HP.choice(.9),
        'decay': HP.choice(1e-4),
        'nesterov': HP.choice(True)
    }

    def __init__(self):
        """
        Sets up the model's internal attributes and loads in saved model hyperparameters when available.
        """

        self.stage_idx = 0
        super(StageOneCalibrator, self).__init__(self.stage_idx)

    def __call__(self, optimizer_params={}, dropouts=[ObjectCalibrator.DEFAULT_DROPOUT]*2, dataset_manager_params={}):
        """
        Method used to build and compile the model represented by this class. Overloads the () operator.

        Parameters
        ----------
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to an empty
            dictionary.
        
        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """

        input_layer = Input(shape=self.input_shape)
        conv_2d = Conv2D(16, (3, 3), activation='relu')(input_layer)
        max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(conv_2d)
        first_dropout = Dropout(dropouts[0])(max_pool_2d)

        flattened = Flatten()(first_dropout)
        fully_connected_layer = Dense(128, activation='relu')(flattened)
        final_dropout = Dropout(dropouts[1])(fully_connected_layer)
        output_layer = Dense(45, activation='softmax')(final_dropout)

        self.model = Model(inputs=input_layer, outputs=output_layer)
        self.compile(optimizer_params)
        return self.model

class StageTwoCalibrator(ObjectCalibrator):
    """
    The calibrator for stage two of the cascade.
    """

    HP = HyperoptWrapper()
    PARAM_SPACE = {
        'dropout0': HP.uniform(0, .75),
        'dropout1': HP.uniform(0, .75),
        'lr': HP.loguniform(1e-4, 1),
        'batchSize': HP.choice(512),
        'norm':  HP.choice(ImageNormalizer.STANDARD_NORMALIZATION),
        'flip': HP.choice(None),
        'momentum': HP.choice(.9),
        'decay': HP.choice(1e-4),
        'nesterov': HP.choice(True)
    }

    def __init__(self):
        """
        Sets up the model's internal attributes and loads in saved model hyperparameters when available.
        """

        self.stage_idx = 1
        super(StageTwoCalibrator, self).__init__(self.stage_idx)

    def __call__(self, optimizer_params={}, dropouts=[ObjectCalibrator.DEFAULT_DROPOUT]*2, dataset_manager_params={}):
        """
        Method used to build and compile the model represented by this class. Overloads the () operator.

        Parameters
        ----------
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to an empty
            dictionary.
        
        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """

        input_layer = Input(shape=self.input_shape)
        conv_2d = Conv2D(32, (5, 5), activation='relu')(input_layer)
        max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(conv_2d)
        first_dropout = Dropout(dropouts[0])(max_pool_2d)

        flattened = Flatten()(first_dropout)
        fully_connected_layer = Dense(64, activation='relu')(flattened)
        final_dropout = Dropout(dropouts[1])(fully_connected_layer)
        output_layer = Dense(45, activation='softmax')(final_dropout)

        self.model = Model(inputs=input_layer, outputs=output_layer)
        self.compile(optimizer_params)
        return self.model

class StageThreeCalibrator(ObjectCalibrator):
    """
    The calibrator for stage three of the cascade.
    """

    HP = HyperoptWrapper()
    PARAM_SPACE = {
        'dropout0': HP.uniform(0, .75),
        'dropout1': HP.uniform(0, .75),
        'lr': HP.loguniform(1e-9, 1),
        'batch_size': HP.choice(512),
        'norm':  HP.choice(ImageNormalizer.STANDARD_NORMALIZATION),
        'flip': HP.choice(None),
        'momentum': HP.choice(.9),
        'decay': HP.choice(1e-4),
        'nesterov': HP.choice(True)
    }

    def __init__(self):
        """
        Sets up the model's internal attributes and loads in saved model hyperparameters when available.
        """

        self.stage_idx = 2
        self.hp = StageThreeCalibrator.HP
        super(StageThreeCalibrator, self).__init__(self.stage_idx)

    def __call__(self, optimizer_params={}, dropouts=[ObjectCalibrator.DEFAULT_DROPOUT]*3, dataset_manager_params={}):
        """
        Method used to build and compile the model represented by this class. Overloads the () operator.

        Parameters
        ----------
        optimizer_params: dict, optional
            Dictionary of hyperparameters for the model's optimizer, defaults to the optimizer settings saved in the model's 
            parameter file or Keras' defaults if the model was not tuned.
        dropouts: list, optional
            List of dropout percentage values, defaults to the dropout percentage values saved in the model's parameter file or
            ObjectClassifier.DEFAULT_DROPOUT is used for each dropout layer if the model was not tuned.
        dataset_manager_params: dict, optional
            Parameter dictionary for the normalizer's dataset manager if the default parameters should be overriden, defaults to an empty
            dictionary.
        
        Returns
        -------
        out: keras.models.Model
        Returns a keras model. The model returned may or may not be compiled.
        """

        input_layer = Input(shape=self.input_shape)
        conv_2d = Conv2D(64, (5, 5), activation='relu')(input_layer)
        max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(conv_2d)
        first_dropout = Dropout(dropouts[0])(max_pool_2d)
        first_batch_norm = BatchNormalization()(first_dropout)
        secondary_conv_2d = Conv2D(64, (5, 5), activation='relu')(first_batch_norm)
        secondary_max_pool_2d = MaxPooling2D(pool_size=(3,3), strides=2)(secondary_conv_2d)
        secondary_batch_norm = BatchNormalization()(secondary_max_pool_2d)
        second_dropout = Dropout(dropouts[1])(secondary_batch_norm)

        flattened = Flatten()(second_dropout)
        fully_connected_layer = Dense(256, activation='relu')(flattened)
        third_dropout = Dropout(dropouts[2])(fully_connected_layer)
        output_layer = Dense(45, activation='softmax')(third_dropout)

        self.model = Model(inputs=input_layer, outputs=output_layer)
        self.compile(optimizer_params)
        return self.model

# cached classifiers and calibrators (meant to be used globally for better performance since model initialization can be expensive)
MODELS = {False: [StageOneClassifier(), StageTwoClassifier(), StageThreeClassifier()], True: [StageOneCalibrator(), StageTwoCalibrator(), StageThreeCalibrator()]}

# parameters used to simplify access to cached classifier and calibrator instances.
_ModelTypes = namedtuple('ObjectTypes', ['STAGE_ONE_CLASSIFIER', 'STAGE_TWO_CLASSIFIER', 'STAGE_THREE_CLASSIFIER', 'STAGE_ONE_CALIBRATOR', 'STAGE_TWO_CALIBRATOR', 'STAGE_THREE_CALIBRATOR'])
ModelTypes = _ModelTypes(STAGE_ONE_CLASSIFIER=MODELS[False][0], STAGE_TWO_CLASSIFIER=MODELS[False][1], STAGE_THREE_CLASSIFIER=MODELS[False][2],
                         STAGE_ONE_CALIBRATOR=MODELS[True][0], STAGE_TWO_CALIBRATOR=MODELS[True][1], STAGE_THREE_CALIBRATOR=MODELS[True][2])