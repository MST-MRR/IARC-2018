############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
A simple interface to handle the task of training a model.
"""

from __future__ import absolute_import, division, print_function, unicode_literals

from sklearn.metrics import f1_score

from .dataset import ClassifierDataset
from .model import DEFAULT_NUM_EPOCHS

def train(model, dataset_manager, num_epochs=DEFAULT_NUM_EPOCHS, tune=True):
    """
    Train `model` for `num_epochs` epochs on the data speficied by `dataset_manager`

    Parameters
    ----------
    model: subclass of `model.ObjectClassifier` or `model.ObjectCalibrator`
        Model to train.
    dataset_manager: data.DatasetManager
        DatasetManager for the given model
    num_epochs: int, optional
        Number of training epochs, default is `DEFAULT_NUM_EPOCHS`.
    tune: bool, optional
        Whether or not to tune the hyperparameters of the model, default is True.
    
    Notes
    ----------
    If `tune` is True and the model already has a parameter file saved, this function will start to retrain the model without
    performing hyperparameter tuning. If you wish to tune the model's hyperparameters again, you must delete the model's parameter
    file.
    
    Returns
    -------
    None
    """

    labels = dataset_manager.get_labels()
    paths = dataset_manager.get_paths()

    if not model.was_tuned() and tune:
        model.tune(dataset_manager, labels, f1_score)

    with ClassifierDataset(paths[0], paths[1], labels) as dataset:
        X_train, X_test, y_train, y_test = dataset.get_stratified_training_set()
        model.fit(X_train, X_test, y_train, y_test, dataset_manager, num_epochs=num_epochs)