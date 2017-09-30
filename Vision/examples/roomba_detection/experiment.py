############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'''Example which allows experimenting with different colorspaces and threshold ranges for color-based object detection.'''

import os
import pickle
import sys
sys.path.append('../..')

import mrrdt_vision
import cv2
import numpy as np

# color space to threshold the image with
COLOR_SPACE = cv2.COLOR_BGR2LAB
# number of color channels
NUM_CHANNELS = 3
# baseline labels for the trackbars in each colorspace
TRACKBAR_LABELS = {
    cv2.COLOR_BGR2LAB: ['L', 'A', 'B'],
    cv2.COLOR_BGR2YCrCb: ['Y', 'Cr', 'Cb'],
    cv2.COLOR_BGR2HSV: ['H', 'S', 'V']
}
# minimum slider value
THRESHOLD_MIN = 0
# maximum slider value
THRESHOLD_MAX = 255
# resolution to resize images to for better performance on large images
RESIZE_TO = (640, 480)

# folder containing images of the target object
POSITIVE_IMAGE_FOLDER = '/home/christopher/IARC-2018/Vision/examples/roomba_detection/Positives'
# title of the window which displays the detection results.
WINDOW_TITLE = 'Roomba Detection Experimental'

def get_positive_image_paths(pos_img_folder=POSITIVE_IMAGE_FOLDER):
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


class ImageThresholder():
    """
    Utility class for displaying thresholded images with the colorspace given by `COLOR_SPACE`

    Parameters
    ----------
    title: str
        Used as both the title and the name of the created window.
    img_paths: sequence of str, optional
        Collection of file paths to each positive image instance in `POSITIVE_IMAGE_FOLDER`.
    """

    def __init__(self, title=WINDOW_TITLE, img_paths=get_positive_image_paths()):
        """
        Saves arguments for the window's construction and initializes internal attributes.
        """

        self.title = title
        self.idx = 0
        self.img_paths = img_paths
        self.quit = False
        self.trackbar_positions = np.zeros(NUM_CHANNELS*2)

        self.set_cur_img()

    def __enter__(self):
        """
        Creates a new window when this object is instantiated by a context manager.
        """

        self.window = mrrdt_vision.Window(self.title).__enter__()
        def on_change(val, idx):
            self.trackbar_positions[idx]=val
        
        for i, trackbar_label in enumerate(TRACKBAR_LABELS[COLOR_SPACE]):
            self.window.create_trackbar(trackbar_label+'_min', THRESHOLD_MIN, THRESHOLD_MAX, on_change=lambda val, idx=i: on_change(val, idx))
            self.window.create_trackbar(trackbar_label, THRESHOLD_MIN, THRESHOLD_MAX, on_change=lambda val, idx=len(self.trackbar_positions)//2+i: on_change(val, idx))

        return self

    def __exit__(self, *args):
        """
        Destroys the window created earlier whenever the context manager ends or encounters an exception.
        """

        self.window.__exit__()

    def set_cur_img(self):
        """
        Select the image referred to by self.idx and display the original image in another window.
        """

        resized_img = cv2.resize(cv2.imread(self.img_paths[self.idx]), RESIZE_TO)
        self.cur_img = cv2.cvtColor(resized_img, COLOR_SPACE)
        cv2.imshow('original', resized_img)
    
    def update(self):
        """
        Display the currently selected image with the user selected threshold values and respond
        to keypresses.

        Returns
        -------
        out: bool
        Returns False if the user wants to quit, True otherwise.
        """

        cur_img_copy = self.cur_img.copy()
        lower_bound = np.array(self.trackbar_positions[:NUM_CHANNELS])
        upper_bound = np.array(self.trackbar_positions[NUM_CHANNELS:])

        mask = cv2.inRange(cur_img_copy, lower_bound, upper_bound)
        cur_img_copy = cv2.bitwise_and(cur_img_copy, cur_img_copy, mask=mask)
        self.window.show(cur_img_copy)

        key_pressed = self.window.get_key()

        if key_pressed == 'n':
            self.idx += 1
            self.idx = self.idx % len(self.img_paths)
            self.set_cur_img()
        elif key_pressed == 'p':
            self.idx -= 1
            if self.idx < 0:
                self.idx = len(self.img_paths)
            self.set_cur_img()
        elif key_pressed == 'q':
            self.quit = True

        return not self.quit

if __name__ == '__main__':
    with ImageThresholder() as image_thresholder:
        while image_thresholder.update():
            pass