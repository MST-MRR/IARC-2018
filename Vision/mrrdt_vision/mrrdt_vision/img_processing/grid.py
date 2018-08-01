#!/usr/bin/env python3.5
############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2018
# Christopher O'Toole

import dlib
import cv2
import numpy as np
from hough_lines import LineDetector
from timeit import default_timer as timer
from time import sleep

TEST_VIDEO_PATH = '../data/vlc-record-2018-07-28-15h53m07s-v4l2____dev_video3-.avi'

class Grid():
    DEFAULT_WHITE_INTENSITY_LOWER_BOUND = 160
    DEFAULT_CONFIDENCE_THRESHOLD = 150
    DEFAULT_BLUR_KERNEL_SIZE = 13
    BORDER_PERCENTAGE = .10
    THRESHOLD = 25

    def __init__(self, 
                 white_intensity_lower_bound=DEFAULT_WHITE_INTENSITY_LOWER_BOUND,
                 threshold=DEFAULT_CONFIDENCE_THRESHOLD,
                 blur_kernel_size=DEFAULT_BLUR_KERNEL_SIZE):
        
        self._white_intensity_lower_bound = white_intensity_lower_bound
        self._threshold = threshold
        self._blur_kernel_size = blur_kernel_size
        self._line_detector = LineDetector(self._threshold)
        self._text = ''

    def detect_grid_border(self, skeleton):
        percent_width = int(skeleton.shape[1]*Grid.BORDER_PERCENTAGE)
        percent_height = int(skeleton.shape[0]*Grid.BORDER_PERCENTAGE)
        self.left_border_detected = len(np.nonzero(skeleton[:, :percent_width])[0]) <= Grid.THRESHOLD
        self.top_border_detected = len(np.nonzero(skeleton[:percent_height, :])[0]) <= Grid.THRESHOLD
        self.bottom_border_detected = len(np.nonzero(skeleton[-percent_height:, :])[0]) <= Grid.THRESHOLD
        self.right_border_detected = len(np.nonzero(skeleton[:, -percent_width:])[0]) <= Grid.THRESHOLD
        words = ['left', 'top', 'bottom', 'right']
        buffer = []

        for i, flag in enumerate((self.left_border_detected, self.top_border_detected, self.bottom_border_detected, self.right_border_detected)):
            if flag:
                buffer.append(words[i])
        
        self._text = ' '.join(buffer)

    def detect(self, img):
        if (len(img.shape) <= 2):
            raise ValueError('img must have 3 channels')
        
        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(grayscale, self._blur_kernel_size)
        ret, thr = cv2.threshold(blurred, self._white_intensity_lower_bound, 255, cv2.THRESH_BINARY)
        skeleton = dlib.skeleton(thr)
        cv2.imshow('skeleton', skeleton)
        self.detect_grid_border(skeleton)
        self._line_detector.detect(skeleton)

    def draw(self, img):
        self._line_detector.draw(img)
        cv2.putText(img, self._text, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

if __name__ == '__main__':
    import sys
    sys.path.append('..')
    from obj_detect.display import VideoReader
    from obj_detect.display import Window
    WINDOW_TITLE = 'Gridline Detection'

    try:
        with VideoReader(TEST_VIDEO_PATH) as test_video, Window(WINDOW_TITLE) as window:
            grid = Grid()
            time = timer()
            number_of_frames = 0
            title = window.get_title()

            for frame in test_video:
                grid.detect(frame)  
                grid.draw(frame)
                window.show(frame)
                if window.is_key_down('q'):
                    break
                
                number_of_frames += 1

                if timer() - time >= 1:
                    time = timer()
                    window.set_title('%s [%d FPS]' % (title, number_of_frames))
                    number_of_frames = 0
    except KeyboardInterrupt:
        pass