#!/usr/bin/env python3.5
############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2018
# Christopher O'Toole

import cv2
import numpy as np
from sklearn.cluster import KMeans

def get_max_line_length_for_image(img):
    return np.sqrt(img.shape[0]**2 + img.shape[1]**2)

'''Utility for detecting lines in an image using the Hough Line Transform'''
class LineDetector():
    RHO = 1
    THETA = np.pi/180
    
    BLUE = (255, 0, 0)
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)

    DEFAULT_COLOR = RED
    DEFAULT_THICKNESS = 2
    DEFAULT_RANDOM_STATE = 42

    '''
    Wrapper around the OpenCV implementation of the Hough Line Transform algorithm
    
    Parameters
    ----------
    name: str

    ''' 
    def __init__(self, threshold, rho=RHO, theta=THETA):
        if (threshold < 0):
            raise ValueError('threshold must be > 0')
        if (rho <= 0):
            raise ValueError('rho must be > 0')
        if (theta <= 0):
            raise ValueError('theta must be > 0')
        
        self._threshold = threshold
        self._rho = rho
        self._theta = theta
        self._lines_group_a = None
        self._lines_group_b = None

    def detect(self, img, cluster=False):
        if (len(img.shape) > 2):
            raise ValueError('img must be grayscale')
        
        max_len = get_max_line_length_for_image(img)
        lines = cv2.HoughLines(img, self._rho, self._theta, self._threshold)
        
        if lines is not None:
            lines = lines[:, 0, :]
            rho, theta = lines[:, 0].reshape(-1, 1), lines[:, 1].reshape(-1, 1)
            ihat, jhat = np.cos(theta), np.sin(theta)
            lines = np.tile(np.hstack((ihat*rho, jhat*rho)), 2)
            lines[:, 0] -= max_len * jhat[:, 0]
            lines[:, 1] += max_len * ihat[:, 0]
            lines[:, 2] += max_len * jhat[:, 0]
            lines[:, 3] -= max_len * ihat[:, 0]
            
         
        self._lines = lines
        self._lines_group_a = None
        self._lines_group_b = None

        if cluster:
            self.cluster()
        
    def cluster(self):
        if self._lines is not None and len(self._lines) > 2:
            slopes = (self._lines[:, 1] - self._lines[:, 3])/(self._lines[:, 0] - self._lines[:, 2] + 1e-9)
            kmeans = KMeans(n_clusters=2, random_state=LineDetector.DEFAULT_RANDOM_STATE).fit(slopes.reshape(-1, 1))
            self._lines_group_a = self._lines[np.where(kmeans.labels_ > 0)]
            self._lines_group_b = self._lines[np.where(kmeans.labels_ == 0)]

    def draw(self, img, color=DEFAULT_COLOR, thickness=DEFAULT_THICKNESS):
        def draw_helper(lines, color=LineDetector.DEFAULT_COLOR, thickness=LineDetector.DEFAULT_THICKNESS):
            for line in lines:
                x1, y1, x2, y2 = line
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)
        
        if (len(img.shape) <= 2):
            raise ValueError('img must have 3 channels')
        
        if self._lines_group_a is not None and self._lines_group_b is not None:
            draw_helper(self._lines_group_a, LineDetector.BLUE)
            draw_helper(self._lines_group_b, LineDetector.RED)
        elif self._lines is not None:
            draw_helper(self._lines)

