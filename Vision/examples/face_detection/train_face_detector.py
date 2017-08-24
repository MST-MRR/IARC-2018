############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'''Example showing how to retrain one of the networks in the face detection cascade with your own dataset.'''

import os
import sys
sys.path.append('../..')

import sqlite3
import datetime

import h5py
import cv2
import numpy as np
import mrrdt_vision
from mrrdt_vision.util import static_vars

# number of pyramid levels to use for object detection
NUM_PYRAMID_LEVELS = 1

# folder containing the SQL database for object annotations
POSITIVE_IMAGE_DATABASE_FOLDER = 'aflw/data/'
# folder containing positive images
POSITIVE_IMAGE_FOLDER = POSITIVE_IMAGE_DATABASE_FOLDER + 'flickr/'
# SQL database file containing the object annotations
POSITIVE_IMAGE_DATABASE_FILE = os.path.join(POSITIVE_IMAGE_DATABASE_FOLDER, 'aflw.sqlite')
# path to each classifier's positive dataset file
OBJECT_DATABASE_PATHS = ('data/pos12.hdf', 'data/pos24.hdf', 'data/pos48.hdf')
# path to folder which contains files that list test image locations
TEST_IMAGE_DATABASE_FOLDER = 'Annotated Faces in the Wild/FDDB-folds/'
# base folder for test images
TEST_IMAGES_FOLDER = 'Annotated Faces in the Wild/originalPics'

# path to folder containing negative images
NEGATIVE_IMG_FOLDER = '/home/christopher/IARC-2018/examples/face_detection/Negative Images/images'
# image file extension to use for negative images
NEGATIVE_IMG_FILE_EXT = '.jpg'
# base folder for face detection cascade files
BASE_FOLDER = 'face'

@static_vars(faces=[])
def get_face_annotations(db_path=POSITIVE_IMAGE_DATABASE_FILE, pos_img_folder=POSITIVE_IMAGE_FOLDER):
    """
    Retrieve face annotation data from an SQL database.

    Parameters
    ----------
    db_path: str, optional
        Path to the SQL database file, defaults to `POSITIVE_IMAGE_DATABASE_FILE`.
    pos_img_folder: str, optional
        Path to the folder which contains the images referenced by the SQL database, defaults to `POSITIVE_IMAGE_FOLDER`.
    Returns
    -------
    out: list
    Returns a list with entries of the form (<image path>, <x>, <y>, <width>, <height>)
    """

    if len(get_face_annotations.faces) == 0:
        with sqlite3.connect(db_path) as conn:
            c = conn.cursor()

            select_string = "faceimages.filepath, facerect.x, facerect.y, facerect.w, facerect.h"
            from_string = "faceimages, faces, facepose, facerect"
            where_string = "faces.face_id = facepose.face_id and faces.file_id = faceimages.file_id and faces.face_id = facerect.face_id"
            query_string = "SELECT " + select_string + " FROM " + from_string + " WHERE " + where_string

            for row in c.execute(query_string):
                img_path = os.path.join(pos_img_folder, str(row[0]))

                if os.path.isfile(img_path):
                    get_face_annotations.faces.append((img_path,) + (row[1:]))

    return get_face_annotations.faces

def get_test_image_paths(test_img_db_folder=TEST_IMAGE_DATABASE_FOLDER, test_imgs_folder=TEST_IMAGES_FOLDER):
    """
    Gets a list of test image paths

    Parameters
    ----------
    test_img_db_folder: str, optional
        Path to folder whose files contain the locations of the test images, defaults to `TEST_IMAGE_DATABASE_FOLDER`.
    test_imgs_folder: str, optional
        Path to the folder that contains the test images, defaults to `TEST_IMAGES_FOLDER`.
    
    Returns
    -------
    out: list
    Returns a list of test image paths
    """

    img_paths = []

    for file_name in os.listdir(test_img_db_folder):
        if 'ellipse' not in file_name:
            with open(os.path.join(test_img_db_folder, file_name)) as in_file:
                img_paths.extend(in_file.read().splitlines())

    return [os.path.join(test_imgs_folder, img_path) + '.jpg' for img_path in img_paths]

def extract_region(img, bounding_box):
    """
    Extract the region described by `bounding_box`.

    Parameters
    ----------
    img: np.ndarray
        Image to extract the region from
    bounding_box: sequence of int
        Sequence containing the coordinates for `bounding_box`. Should be of the form (x_min, y_min, x_max, y_max)

    Returns
    -------
    out: numpy.ndarray
    Returns the extracted image region.
    """
    
    x_min, y_min, x_max, y_max = bounding_box
    return img[y_min:y_max, x_min:x_max]

def expand_negative_dataset_with_negatives_from_alfw(negative_img_folder=NEGATIVE_IMG_FOLDER, num_pyramid_levels=NUM_PYRAMID_LEVELS):
    """
    Expand the a negative dataset with samples taken from Annotated Faces in the Wild. This function should be used with caution,
    since some face instances are not annotated and it's possible that those images will be put in the negative image folder as well.

    Parameters
    ----------
    negative_img_folder: str, optional
        The path to the negative image folder to add the images to, defaults to `DEFAULT_IMG_FOLDER`.
    num_pyramid_levels: int, optional
        The number of image pyramid levels to use for the detector, defaults to `NUM_PYRAMID_LEVELS`.

    Returns
    -------
    None
    """

    prev_img_path = None
    cur_img = None
    face_bounding_boxes_for_cur_img = []

    for img_path, x, y, w, h in get_face_annotations():
        if img_path != prev_img_path:
            if prev_img_path is not None:
                face_bounding_boxes_for_cur_img = np.asarray(face_bounding_boxes_for_cur_img)
                detected_faces = mrrdt_vision.detect_object(cur_img, mrrdt_vision.ObjectTypes.FACE, num_pyramid_levels=num_pyramid_levels)

                for detected_face in detected_faces:
                    if np.allclose(mrrdt_vision.iou(face_bounding_boxes_for_cur_img, detected_face), 0):
                        timestamp = str(datetime.datetime.now().time())
                        cv2.imwrite(os.path.join(negative_img_folder, timestamp + NEGATIVE_IMG_FILE_EXT), extract_region(cur_img, detected_face))

                face_bounding_boxes_for_cur_img = []
            
            cur_img = cv2.imread(img_path)
            prev_img_path = img_path

        x, y, w, h = mrrdt_vision.squash_coords(cur_img, x, y, w, h)
        face_bounding_boxes_for_cur_img.append((x, y, x+w, y+h))

if __name__ == '__main__':
    classifier = mrrdt_vision.ModelTypes.STAGE_THREE_CLASSIFIER
    dataset_manager = mrrdt_vision.DatasetManager(classifier, get_face_annotations, BASE_FOLDER, POSITIVE_IMAGE_FOLDER, NEGATIVE_IMG_FOLDER)
    mrrdt_vision.train(classifier, dataset_manager)
