############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'''Example showing how to use the mrrdt_vision module to detect roombas in images.'''

import sys
import os
sys.path.append('../..')

import mrrdt_vision
import cv2

# folder containing images of roombas.
POSITIVE_IMAGE_FOLDER = '/home/christopher/IARC-2018/examples/roomba_detection/Positives'
# title of the window which displays the detection results.
WINDOW_TITLE = 'Roomba Detector Test'
# condifidence threshold for the roomba detector's classifier
STAGE_ONE_THRESH = .1
# number of times to apply the calibrator to the bounding box proposals
NUM_CALIB_STEPS = 1

def get_roomba_image_paths(pos_img_folder=POSITIVE_IMAGE_FOLDER):
    """
    Retrieves the path of every image file in the directory described by `pos_img_folder`.

    Parameters
    ----------
    pos_img_folder: str, optional
        Path to the folder which containing the targeted images, defaults to `POSITIVE_IMAGE_FOLDER`.

    Returns
    -------
    out: list
    Returns a list of paths that correspond to every file in `pos_img_folder`.
    """

    return [os.path.join(pos_img_folder, file_name) for file_name in os.listdir(pos_img_folder)]

def callback(img):
    """
    Visualizer callback that draws bounding boxes around where the detector has predicted roombas are in
    the image.

    Parameters
    ----------
    img: numpy.ndarray
        The image that will be processed by the detector.

    Notes
    -------
    This function will potentially modify `img` by drawing a bounding box around any roomba
    the detector sees in the image. 

    Returns
    -------
    None
    """
    detections, centers = mrrdt_vision.detect_object(img, mrrdt_vision.ObjectTypes.ROOMBA, stage_one_thresh=STAGE_ONE_THRESH, num_calibration_steps=NUM_CALIB_STEPS)
    mrrdt_vision.draw_bounding_boxes(img, detections)
    
def main():
    """
    Graphically visualizes the results of the roomba detector on samples that are known to be positive.
    """
    mrrdt_vision.visualizer(get_roomba_image_paths(), callback, WINDOW_TITLE)

if __name__ == '__main__':
    main()
