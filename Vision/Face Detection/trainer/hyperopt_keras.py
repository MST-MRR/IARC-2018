############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
A small library to make using keras, sklearn, and hyperopt together for hyperparameter tuning eaiser.

Credit to [1] for providing the mathematical magic that makes my hyperparameter tuning functions work.

References
----------
.. [1] Bergstra, J., Yamins, D., & Cox, D. D. (2013). Hyperopt: A python library for optimizing the hyperparameters of machine learning algorithms. 
       In Proceedings of the 12th Python in Science Conference (pp. 13-20).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os
import numpy as np
import atexit

from hyperopt import hp, fmin, tpe, STATUS_OK, STATUS_FAIL, Trials, space_eval

# default number of folds for model cross-validation
DEFAULT_NUM_FOLDS = 3
# default number of epochs per parameter set evaluation
DEFAULT_NUM_EPOCHS = 30
# default number of parameter sets tried
DEFAULT_NUM_EVALS = 30
# a temporary weights file used to evaluate each parameter set fold
WEIGHTS_FILE_NAME = 'tune.hdf'

# num evals parameter name for tune function
NUM_EVALS_PARAM_NAME = 'num_evals'

class HyperoptWrapper():
    """
    Lightweight wrapper for the hyperopt library

    Parameters
    ----------
    None
    
    Usage
    ----------
    This class will handle labelling each parameter for you, but the simplicity of the labelling algorithm
    may lead to hard to diaganose bugs. Therefore, it is recommended that you create a new HyperoptWrapper instance
    for each parameter space used. You must also completely retune the model when you add new parameters

    Example
    --------
    >>> HP = HyperoptWrapper()
    >>> PARAM_SPACE = {
    ...    'dropout0': HP.uniform(0, .75),
    ...    'dropout1': HP.uniform(0, .75),
    ...    'lr': HP.loguniform(1e-4, 1),
    ...    'batchSize': HP.choice(512),
    ...    'norm':  HP.choice(ImageNormalizer.ZCA_WHITENING),
    ...    'flip': HP.choice(None),
    ...    'momentum': HP.choice(.9),
    ...    'decay': HP.choice(1e-4),
    ...    'nesterov': HP.choice(True)
    ... }

    """
    
    def __init__(self):
        self.label_idx = 0

    def _get_param(self, func, *args):
        """
        Get a hyperopt parameter object by passing to `func` the distribution range specified by `*args `

        Parameters
        ----------
        func: callable
            Function to construct hyperopt parameter object with
        *args: sequence of objects
            Sequence specifying the range of values we can sample from for this hyperparameter

        Returns
        -------
        out: hyperopt.pyll.base.Apply
        A hyperopt parameter object
        """

        param = func(str(self.label_idx), *args)
        self.label_idx += 1
        return param

    def uniform(self, low, high):
        """
        Get a hyperopt parameter object that samples floating point values from a uniform continuous random distribution

        Parameters
        ----------
        low: float
            Lower bound
        high: float
            Upper bound

        Notes
        ----------
        A uniform distribution implies that each value in the specified range has an equal chance of being chosen
        
        Returns
        -------
        out: hyperopt.pyll.base.Apply
        A hyperopt parameter object that samples floating point values from the interval [low, high]
        """

        return self._get_param(hp.uniform, low, high)

    def choice(self, *args):
        """
        Get a hyeropt parameter object that samples from a finite set of values
        
        Parameters
        ----------
        *args: sequence of objects
            Sequence to sample from

        Notes
        ----------
        Each value in the sequence has an equal chance of being chosen
        
        Returns
        -------
        out: hyperopt.pyll.base.Apply
        A hyperopt parameter object that samples from the sequence specified in `*args`
        """
        
        return self._get_param(hp.choice, list(args))

    def randint(self, low, high):
        """
        Get a hyperopt parameter object that samples random integer values uniformly from a specified range

        Parameters
        ----------
        low: float
            Lower bound
        high: float
            Upper bound

        Notes
        ----------
        Each integer within the given range has an equal chance of being chosen
        
        Returns
        -------
        out: hyperopt.pyll.base.Apply
        A hyperopt parameter object that samples integers from the interval [low, high]
        """

        return low + self._get_param(hp.randint, max(low, high - low))

    def loguniform(self, low, high):
        """
        Get a hyperopt parameter object that samples floating point values from a log uniform continuous random distribution

        Parameters
        ----------
        low: float
            Lower bound
        high: float
            Upper bound

        Notes
        ----------
        This distribution will return values that are uniformly distributed on a logarithmic scale. However, on
        a non-logarithmic scale values closer to `low` have a higher chance of being chosen.
        
        Returns
        -------
        out: hyperopt.pyll.base.Apply
        A hyperopt parameter object that samples floating point values from the interval [low, high]
        """

        return self._get_param(hp.loguniform, np.log(low), np.log(high))


def parse_params(params):
    """
    Split up model hyperparameters into groups

    Parameters
    ----------
    params: dict
        Dictionary of model hyperparameters
    
    Returns
    -------
    out: dict, dict, dict, list
    Returns normalization parameter dictionary, optimizer parameter dictionary, training parameter dictionary, and list of dropouts
    """

    from .model import DROPOUT_PARAM_ID, OPTIMIZER_PARAMS, NORMALIZATION_PARAMS, TRAIN_PARAMS

    normalization_params = {}
    optimizer_params = {}
    train_params = {}
    dropouts = []

    for k, v in params.items():
        if k in NORMALIZATION_PARAMS:
            normalization_params[k] = v
        elif k in OPTIMIZER_PARAMS:
            optimizer_params[k] = v
        elif k in TRAIN_PARAMS:
            train_params[k] = v
        elif type(k) is str and k.startswith(DROPOUT_PARAM_ID):
            if not dropouts:
                dropouts.append(v)
            else:
                idx = int(k.replace(DROPOUT_PARAM_ID, ''))
                dropouts.insert(idx, v)

    return normalization_params, optimizer_params, train_params, dropouts


def optimize(func):
    """
    Tunes the hyperparameters of a classifier with `num_folds` cross-validation and `num_evals` trials via trees of parzen estimators and
    stratified sampling.

    Parameters
    ----------
    func: callable
        This function should evaluate the set of hyperparameters given to it and return the results.
        See https://github.com/hyperopt/hyperopt/wiki/FMin for more information.

    Usage
    ----------
    >>> @optimize
    ... def func(...)
    ...   # evaluate the hyperparameter set passed
    """

    def decorate(param_space, *args, **kwargs):
        """
        Tunes hyperparameters in the given `param_space` via hyperopt's `fmin` function

        Parameters
        ----------
        param_space: dict
            Dictionary describing the model's hyperparameter space
        *args: sequence of objects
            Sequence of objects to pass to `func` every evaluation
        **kwargs: dict, optional
            Contains any keyword agrguments. `num_evals` is the number of parameter sets to evaluate, defaults to `DEFAULT_NUM_EVALS`.
        
        Returns
        -------
        out: dict, hyperopt.Trials
        Returns the best hyperparameter set found and a Trials object which contains information about the entire hyperparameter tuning run.
        """

        num_evals = kwargs.get(NUM_EVALS_PARAM_NAME) or DEFAULT_NUM_EVALS

        if NUM_EVALS_PARAM_NAME in kwargs.keys():
            del kwargs[NUM_EVALS_PARAM_NAME]

        trials = Trials()

        atexit.register(os.remove, WEIGHTS_FILE_NAME)
        best = fmin(lambda params: func(params, *args, **kwargs), param_space, algo=tpe.suggest, max_evals=num_evals, trials=trials)
        return best, trials

    return decorate


@optimize
def tune(params, model, dataset_manager, labels, metric, num_folds=DEFAULT_NUM_FOLDS, num_epochs=DEFAULT_NUM_EPOCHS, verbose=True):
    """
    Evaluates model hyperparameters via `num_folds` cross-validation

    Parameters
    ----------
    params: dict
        Hyperparameters to evaluate
    model: subclass of `model.ObjectClassifier` or `model.ObjectCalibrator`
        Model to tune
    dataset_manager: data.DatasetManager
        DatasetManager for the given model
    labels: numpy.ndarray
        Class labels for each dataset sample
    metric: callable
        Scikit-learn metric to optimize
    num_folds: int, optional
        Number of folds to use for cross-validation, default is `DEFAULT_NUM_FOLDS`.
    num_epochs: int, optional
        Number of training epochs for each cross-validation fold, default is `DEFAULT_NUM_EPOCHS`.
    num_evals: int, optional
        Number of distinct parameter sets to evaluate, default is `DEFAULT_NUM_EVALS`.
    verbose: bool, optional
        Controls whether or not the parameters and results of each trial is shown, default is True.
    
    Returns
    -------
    out: dict
    Returns a dictionary containing the metric average, metric variance, and status of this evaluation.

    See Also
    ----------
    `hyperopt_keras.optimize`
    """

    from .dataset import ClassifierDataset
    from .preprocess import ImageNormalizer

    log = lambda *args: print(*args) if verbose else None

    normalization_params, optimizer_params, train_params, dropouts = parse_params(params)
    batch_size = train_params['batch_size']
    norm_method = normalization_params['norm']
    del normalization_params['norm']

    paths = dataset_manager.get_paths()

    with ClassifierDataset(paths[0], paths[1], labels) as dataset:
        scores = []

        normalizer = ImageNormalizer(dataset_manager.get_pos_dataset_file_path(), dataset_manager.get_neg_dataset_file_path(), norm_method)
        normalizer.add_data_augmentation_params(normalization_params)
        fit_params = {'optimizer_params': optimizer_params, 'dropouts': dropouts,
                     'batch_size': batch_size, 'verbose': False, 'save_file_path': WEIGHTS_FILE_NAME}

        for i, (X_train, X_test, y_train, y_test) in enumerate(dataset.stratified_splitter(num_folds)):
            model.fit(X_train, X_test, y_train, y_test, dataset_manager, normalizer, num_epochs, **fit_params)

            if not i:
                log('Using params:', params)
            
            metric_params = {'average': 'binary' if np.amax(y_test) == 1 else 'macro'}
            scores.append(model.eval(X_test, y_test, normalizer, metric, weights_file_path=WEIGHTS_FILE_NAME, dataset_manager=dataset_manager, **metric_params))
            log('Fold %d score: %.5f' % (i, scores[-1]))

        metric_avg = np.mean(np.array(scores), dtype=np.float64)
        loss_variance = np.var(np.array(scores), dtype=np.float64)
        log('Average score: %.5f' % (metric_avg,))
        log('Variance: %.5f' % (loss_variance,))

    sys.stdout.flush()

    return {'loss': -metric_avg, 'loss_variance': loss_variance, 'status': STATUS_OK}

def get_best_params(param_space, best):
    """
    Helps retrieve the best hyperparameter set from a saved `hyperopt.Trials` object.

    Parameters
    ----------
    param_space: dict
        Dictionary describing the model's hyperparameter space.
    best: dict
        Best set of hyperparameter values stored in the `hyperopt.Trials` object.
    
    Returns
    -------
    out: dict
    Returns the best hyperparameter set in `param_space` for a particular model
    """

    return space_eval(param_space, {k: v[0] if type(v) is list else v for k, v in best.items()})
