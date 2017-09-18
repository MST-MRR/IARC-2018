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
import numpy as np
import cv2

# path to IARC footage
IARC_VIDEO = 'IARC.mp4'
# title of the window which displays the detection results.
WINDOW_TITLE = 'Roomba Detector Test'
# condifidence threshold for the roomba detector's classifier
THRESH = .001
# number of times to apply the calibrator to the bounding box proposals
NUM_CALIB_STEPS = 1
# resolution to resize images to for better performance on large images
RESIZE_TO = (640, 480)
# key to press to quit
QUIT_KEY = 'q'

# lower bound for the red roomba's flap in YCrCb space
RED_ROOMBA_YCRCB_LOWER_BOUND = np.array([0, 156, 107])
# upper bound for the red roomba's flap in YCrCb space
RED_ROOMBA_YCRCB_UPPER_BOUND = np.array([255, 255, 255])
# lower bound for the green roomba's flap in LAB space
GREEN_ROOMBA_LAB_LOWER_BOUND = np.array([0, 0, 127])
# upper bound for the green roomba's flap in LAB space
GREEN_ROOMBA_LAB_UPPER_BOUND = np.array([94, 123, 250])
# minimum area for a region to have a chance at being considered a roomba
MIN_ROOMBA_AREA = 100

def _get_roomba_proposals(img):
    proposals = []
    centers = []

    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    ycrcb_img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    red_roomba_mask = cv2.inRange(ycrcb_img, RED_ROOMBA_YCRCB_LOWER_BOUND, RED_ROOMBA_YCRCB_UPPER_BOUND)
    green_romba_mask = cv2.inRange(lab_img, GREEN_ROOMBA_LAB_LOWER_BOUND, GREEN_ROOMBA_LAB_UPPER_BOUND)
    
    combined_mask = np.bitwise_or(red_roomba_mask, green_romba_mask)
    opened_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, np.ones((5, 5), dtype=np.uint8))
    closed_mask = cv2.morphologyEx(opened_mask, cv2.MORPH_CLOSE, np.ones((11, 11), dtype=np.uint8))

    modified_img, contours, hierarchy = cv2.findContours(closed_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= MIN_ROOMBA_AREA:
            moments = cv2.moments(contour)
            centers.append((int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])))
            x, y, w, h = cv2.boundingRect(contour)
            dimensions = np.array([w, h])
            top_left = np.array([x, y]) - MIN_ROOMBA_AREA//4
            bottom_right = np.array([x, y]) + dimensions + MIN_ROOMBA_AREA//4
            x_min, y_min = top_left.astype(int)
            x_max, y_max = bottom_right.astype(int)
            proposals.append((x_min, y_min, x_max, y_max))

    return proposals, centers

def main():
    """Qualitative display of the roomba detector's results on a video clip from the IARC 2017 arena"""

    with mrrdt_vision.VideoReader(IARC_VIDEO) as video, mrrdt_vision.Window(WINDOW_TITLE) as window:
        try:
            should_quit = False

            for frame in video:
                if should_quit:
                    break
                if frame is None:
                    continue

                frame = cv2.resize(frame, RESIZE_TO)
                detections, centers = _get_roomba_proposals(frame)
                mrrdt_vision.draw_bounding_boxes(frame, detections)

                window.show(frame)

                should_quit = window.is_key_down(QUIT_KEY)
        except KeyboardInterrupt as e:
            pass

if __name__ == '__main__':
    main()
