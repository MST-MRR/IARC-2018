############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Module with utilities for objectively evaluating the effectiveness of a Keras classifier via a varierty of metrices.
"""

from __future__ import absolute_import, division, print_function, unicode_literals

import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import precision_recall_curve, confusion_matrix, f1_score, precision_score, recall_score, roc_curve, roc_auc_score

from .dataset import ClassifierDataset
from .model import ObjectCalibrator
from .detect import THRESHOLDS

class ModelEvaluator():
    """
    Evaluates the effectiveness of a Keras classifier.

    Parameters
    ----------
    model: subclass of `model.ObjectClassifier` or `model.ObjectCalibrator`
        Model to evaluate.
    dataset_manager: data.DatasetManager
        DatasetManager for the evaluation dataset.

    Notes
    ----------
    This class assumes that the dataset_manager will produce the same train/test split used in training in order to
    guarantee that the model is only evaluated on data it didn't see during training.
    """

    def __init__(self, model, dataset_manager):
        """
        Performs the necessary preparation to evaluate `model` on the dataset specified by `dataset_manager`
        """

        self.model = model
        self.dataset_manager = dataset_manager
        self.dataset = ClassifierDataset(*dataset_manager.get_paths(), dataset_manager.get_labels())
        self.is_calib = isinstance(model, ObjectCalibrator)
        self.average = 'macro' if self.is_calib else 'binary'
        self.X_test = None
        self.y_test = None
        self.y_pred = None

    def __enter__(self):
        """
        Saves and predicts on a dataset when this object is instantiated by a context manager.
        """

        try:
            self.dataset.__enter__()
        except:
            self.dataset.__exit__()
            raise
        
        X_train, X_test, y_train, y_test = self.dataset.get_stratified_training_set()
        self.X_test, self.y_test = (X_test, y_test)
        self.predictions = self._predict()
        self.y_pred = np.argmax(self.predictions, axis = 1) if self.is_calib else (self.predictions[:, 1] >= THRESHOLDS[self.model.get_stage_idx()])
        self.y_pred = self.y_pred.reshape(len(y_test), 1)
        return self

    def __exit__(self, *args):
        """
        Frees the dataset the model was evaluated on whenever the context manager ends or encounters an exception.
        """

        self.dataset.__exit__() 

    def _predict(self):
        """
        Get the model's predictions for the dataset's test set.

        Returns
        -------
        out: numpy.ndarray
        Returns the model's predictions for dataset's test set.
        """

        return self.model.predict(self.X_test, self.dataset_manager.get_normalizer(), dataset_manager=self.dataset_manager)

    def confusion_matrix(self, visualize=False):
        """
        Computes and optionally displays graphically the model's confusion matrix.

        Parameters
        ----------
        visualize: bool, optional
            Whether or not to display a graphical representation of the model's confusion matrix, default is False.

        Returns
        -------
        out: numpy.ndarray
        Returns the model's confusion matrix for the test set.
        """

        matrix = confusion_matrix(self.y_test, self.y_pred)
        
        if visualize:
            plt.matshow(matrix, cmap=plt.cm.gray)
            plt.show()

        return matrix

    def f1_score(self):
        """
        Computes the model's f1-score.

        Notes
        ----------
        If the model has more than two class categories as output, the macro average of the model's f1-score
        is returned.

        Returns
        -------
        out: float
        Returns the model's f1-score on the test set.
        """

        return f1_score(self.y_test, self.y_pred, average=self.average)

    def accuracy(self):
        """
        Computes the model's accuracy.

        Returns
        -------
        out: float
        Returns the accuracy of the model's test set predictions.
        """

        return np.sum(np.equal(self.y_test, self.y_pred), dtype=np.float64)/len(self.y_test)

    def precision(self):
        """
        Computes the model's precision.

        Returns
        -------
        out: float
        Returns the precision of the model's test set predictions.
        """

        return precision_score(self.y_test, self.y_pred, average=self.average)

    def recall(self):
        """
        Computes the model's recall.

        Returns
        -------
        out: float
        Returns the model's recall rate for the test set.
        """

        return recall_score(self.y_test, self.y_pred, average=self.average)

    def roc_auc_score(self):
        """
        Computes the area under the model's ROC curve.

        Notes
        ----------
        If the model has more than two class categories as output, the macro average of the model's AUC score
        is returned.

        Returns
        -------
        out: float
        Returns the area under the model's ROC curve for its test set predictions.
        """

        return roc_auc_score(self.y_test, self.y_pred, average=None if self.average == 'binary' else self.average)
    
    def plot_roc_curve(self):
        """
        Displays the model's ROC curve for the test set.

        Returns
        -------
        None
        """

        fpr, tpr, thresholds = roc_curve(self.y_test, self.predictions[:, 1])
        plt.plot(fpr, tpr, linewidth=2)
        plt.plot([0, 1], [0, 1], 'k--')
        plt.axis([0, 1, 0, 1])
        plt.xlabel('False Positive Rate')
        plt.ylabel('True Positive Rate')
        plt.show()

    def plot_precision_recall_curve(self):
        """
        Displays the model's precision-recall curve for the test set.

        Returns
        -------
        None
        """

        precisions, recalls, thresholds = precision_recall_curve(self.y_test, self.predictions[:, 1])
        plt.plot(recalls, precisions, 'b--')
        plt.xlabel('Recall')
        plt.ylabel('Precision')
        plt.ylim([0, 1])
        plt.xlim([0, 1])
        plt.show()

    def plot_precision_recall_vs_threshold(self):
        """
        Displays the model's precision and recall as a function of the model's prediction threshold for the test set.

        Returns
        -------
        None
        """

        precisions, recalls, thresholds = precision_recall_curve(self.y_test, self.predictions[:, 1])
        plt.plot(thresholds, precisions[:-1], 'b--', label='Precision')
        plt.plot(thresholds, recalls[:-1], 'g-', label='Recall')
        plt.xlabel('Threshold')
        plt.legend(loc='upper left')
        plt.ylim([0, 1])
        plt.xlim([0, 1])
        plt.show()

    def summary(self):
        """
        Summarize the model's performance on the test set by displaying evaluation statistics from every applicable metric 
        available and plot some curves to aid in visualizing how well the model generalizes.

        Returns
        -------
        None
        """

        print('\nPrecision:\t\t%.3f%%' % (self.precision()*100,))
        print('Recall:\t\t\t%.3f%%' % (self.recall()*100,))
        print('Accuracy:\t\t%.3f%%' % (self.accuracy()*100,))
        print('f1-score:\t\t%.3f' % self.f1_score())
        if not self.is_calib: print('Area under ROC curve:\t%.3f' % self.roc_auc_score())
        print('\nConfusion Matrix:\n', self.confusion_matrix())
        print()

        if self.is_calib:
            print('Plotting confusion matrix...')
            self.confusion_matrix(visualize = True)
        else:
            print('Plotting ROC curve...')
            self.plot_roc_curve()
            print('Plotting precision-recall curve...')
            self.plot_precision_recall_curve()
            print('Plotting precision and recall vs. threshold curve...')
            self.plot_precision_recall_vs_threshold()
