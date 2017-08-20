############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'''Example demonstrating how to use the mrrdt_vision interface to train a roomba detector.'''

import os
import pickle
import sys
sys.path.append('../..')

import mrrdt_vision
import cv2

# folder containing postive image samples
POSITIVE_IMAGE_FOLDER = '/home/christopher/IARC-2018/examples/roomba_detection/Positives'
# folder containg negative image samples
NEGATIVE_IMAGE_FOLDER = '/home/christopher/IARC-2018/examples/roomba_detection/Negatives'
# binary file containing the annotations for each postive image
ANNOTATIONS_FILE = '/home/christopher/IARC-2018/examples/roomba_detection/Positives/annotations'

# folder containing the files for the roomba detector.
BASE_FOLDER = 'roomba'
# minimum area needed for a bounding box from the annotations file to be considered valid.
MIN_AREA = 144
# whether or not to retrain a model from the roomba detector when it has already been trained
RETRAIN = True

def get_roomba_annotations(annotations_file_path=ANNOTATIONS_FILE, pos_img_folder=POSITIVE_IMAGE_FOLDER):
    """
    Retrieves the roomba annotations from a file created by the "Image Dataset Utility" software provided in
    this repo.

    Parameters
    ----------
    annotations_file_path: str, optional
        Path to the annotations file, defaults to `ANNOTATIONS_FILE`.
    pos_img_folder: str, optional
        Path to the folder which contains the images the annotations file refers to, defaults to `POSITIVE_IMAGE_FOLDER`.
    
    Returns
    -------
    out: list
    Returns a list with entries of the form (<image path>, <x>, <y>, <width>, <height>)
    """

    roombas = []

    with open(annotations_file_path, 'rb') as annotations_file:
        annotations = pickle.load(annotations_file)

        for cur_file_name, cur_file_annotations in annotations.items():
            cur_file_path = os.path.join(pos_img_folder, cur_file_name)

            if os.path.isfile(cur_file_path):
                for annotation in cur_file_annotations:
                    top_x, top_y = tuple(annotation[:,0])
                    bottom_x, bottom_y = tuple(annotation[:, 1])

                    if (abs(top_x-bottom_x)*abs(bottom_y-top_y)>=MIN_AREA):
                        img = cv2.imread(cur_file_path)
                        roombas.append((cur_file_path, top_x, top_y, bottom_x - top_x, bottom_y - top_y))

    return roombas

def train_roomba_detector(retrain=False):
    """
    Trains a classifier and a calibrator for detecting roombas.

    Parameters
    ----------
    retrain: bool, optional
        Whether or not to retrain the models if they've already been trained.
    
    Returns
    -------
    None
    """

    classifier, calibrator = (mrrdt_vision.ModelTypes.STAGE_ONE_CLASSIFIER, mrrdt_vision.ModelTypes.STAGE_ONE_CALIBRATOR)

    for model in (classifier, calibrator):
        dataset_manager = mrrdt_vision.DatasetManager(model, get_roomba_annotations, BASE_FOLDER, POSITIVE_IMAGE_FOLDER, NEGATIVE_IMAGE_FOLDER)

        if not os.path.isfile(model.get_weights_file_path()) or retrain:
            mrrdt_vision.train(model, dataset_manager)

if __name__ == '__main__':
    train_roomba_detector(retrain=RETRAIN)
