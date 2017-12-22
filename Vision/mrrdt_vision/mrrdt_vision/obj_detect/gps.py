import os
import sys
import cv2
import numpy as np
import matplotlib

from matplotlib import pyplot as plt

from sklearn import linear_model, cluster
from sklearn.neighbors.kde import KernelDensity
from scipy.signal import argrelextrema

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import display

TEST_IMAGES_PATH = '../../../../Flight/log/'
TEST_IMAGE_KEYWORD = '.jpg'
TEST_IMAGES = [os.path.join(TEST_IMAGES_PATH, name) for name in os.listdir(TEST_IMAGES_PATH) if TEST_IMAGE_KEYWORD in name]

#TODO: correct number based on frame width
#CAMERA_FOCAL_LENGTH = 623.5365275900853

class Grid:
    KERNEL_BANDWIDTH = 20
    KERNEL_TYPE = 'gaussian'
    MEDIAN_BLUR_SIZE = 5
    THETA_MIN = -np.pi/4
    THETA_MAX = np.pi*3/4
    CANNY_MIN = 100
    CANNY_MAX = 150
    HOUGH_LINE_THETA_ACC = np.pi/180
    HOUGH_LINE_RHO_ACC = 1
    LINE_THRESHOLD = 40
    CLUSTERING_THRESHOLD_SCALAR = .25
    NORMALIZATION_FACTOR = np.sqrt(2*np.pi)

    def __init__(self):
        self._kernel_bandwidth = Grid.KERNEL_BANDWIDTH
        self._kernel_type = Grid.KERNEL_TYPE
        self._median_blur_size = Grid.MEDIAN_BLUR_SIZE
        self._theta_min = Grid.THETA_MIN
        self._theta_max = Grid.THETA_MAX
        self._canny_min = Grid.CANNY_MIN
        self._canny_max = Grid.CANNY_MAX
        self._hough_line_theta_acc = Grid.HOUGH_LINE_THETA_ACC
        self._hough_line_rho_acc = Grid.HOUGH_LINE_RHO_ACC
        self._line_threshold = Grid.LINE_THRESHOLD
        self._clustering_threshold_scalar = Grid.CLUSTERING_THRESHOLD_SCALAR
        self._normalization_factor = Grid.NORMALIZATION_FACTOR

        self._latitudinal_lines = None
        self._longitudinal_lines = None
        self._latitudinal_rho = None
        self._longitudinal_rho = None
        self._latitudinal_theta = None
        self._longitudinal_theta = None

    @property
    def latitudinal_lines(self):
        return self._latitudinal_lines

    @property
    def longitudinal_lines(self):
        return self._longitudinal_lines

    def _preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, self._median_blur_size)
        thresholded_img = cv2.bitwise_and(blur, blur, mask=cv2.inRange(blur, 245, 255))
        edges = cv2.Canny(thresholded_img, self._canny_min, self._canny_max)
        cv2.imshow('blur', blur)
        cv2.imshow('edges', edges)
        return edges
    
    def _hough_line_transform(self, edges):
        return cv2.HoughLines(edges, self._hough_line_rho_acc, self._hough_line_theta_acc, self._line_threshold)

    def _ransac2d(self, data, X, y):
        ransac = linear_model.RANSACRegressor()
        ransac.fit(X.reshape(-1, 1), y.reshape(-1, 1))
        return data[ransac.inlier_mask_], X[ransac.inlier_mask_], y[ransac.inlier_mask_], np.logical_not(ransac.inlier_mask_)
    
    def _filter_lines(self, lines):
        rho = lines[:, 0, 0]
        theta = lines[:, 0, 1]
        idxs = np.where(np.bitwise_and(self._theta_min <= theta, theta <= self._theta_max))
        rho, theta, lines = (rho[idxs], theta[idxs], lines[idxs])

        lines_set_A, rho_A, theta_A, outliers_idxs = self._ransac2d(lines, rho, theta)
        lines_set_B, rho_B, theta_B, outliers_idxs = self._ransac2d(lines[outliers_idxs], rho[outliers_idxs], theta[outliers_idxs])
        is_A_longitudinal = np.mean(theta_A) < np.mean(theta_B)
        
        if is_A_longitudinal:
            self._longitudinal_lines, self._longitudinal_rho, self._longitudinal_theta = (lines_set_A, rho_A, theta_A)
            self._latitudinal_lines, self._latitudinal_rho, self._latitudinal_theta = (lines_set_B, rho_B, theta_B)
        else:
            self._longitudinal_lines, self._longitudinal_rho, self._longitudinal_theta = (lines_set_B, rho_B, theta_B)
            self._latitudinal_lines, self._latitudinal_rho, self._latitudinal_theta = (lines_set_A, rho_A, theta_A)

    def _cluster(self, lines, rho):
        clusters = []
        kde = KernelDensity(bandwidth=self._kernel_bandwidth, kernel=self._kernel_type).fit(rho.reshape(-1, 1))
        support = np.linspace(rho.min(), rho.max())
        densities = kde.score_samples(support.reshape(-1, 1))/(len(rho)*self._kernel_bandwidth*self._normalization_factor)
        densities = densities[np.where(densities < self._clustering_threshold_scalar*densities.max())]
        minima = support[argrelextrema(densities, np.less)[0]]
    
        idxs_list = [(rho >= minima[i]) * (rho <= minima[i+1]) for i in range(len(minima)-1)]
        idxs_list.insert(0, rho <= minima[0])
        idxs_list.append(rho >= minima[-1])
        
        for idxs in idxs_list:
            if len(idxs) > 0:
                clusters.append(np.mean(lines[idxs, 0, :], axis=0))

        return np.asarray(clusters)
    
    def detect(self, img):
        lines = self._hough_line_transform(self._preprocess(img))

        if lines is not None:
            self._filter_lines(lines)
            self._draw_lines(lines[:, 0, :], img, color=(255, 0, 255))
            self._draw_lines(self.longitudinal_lines[:, 0, :], img, color=(255, 0, 0))
            self._draw_lines(self.latitudinal_lines[:, 0, :], img, color=(0, 255, 0))
            self._longitudinal_lines = self._cluster(self.longitudinal_lines, self._longitudinal_rho)
            self._latitudinal_lines = self._cluster(self.latitudinal_lines, self._latitudinal_rho)
            self._longitudinal_rho, self._longitudinal_theta = (self.longitudinal_lines[:, 0], self.longitudinal_lines[:, 1])
            self._latitudinal_rho, self._latitudinal_theta = (self.latitudinal_lines[:, 0], self.latitudinal_lines[:, 1])
    
    def _draw_lines(self, lines, img, color=(0, 0, 0), thickness=2):
        max_dist = np.sqrt(np.sum(np.asarray(img.shape[:2])**2))

        for rho, theta in lines:
            scalar = np.array([np.cos(theta), np.sin(theta)])
            point0 = scalar*rho
            offset = max_dist*np.array([-scalar[1], scalar[0]])
            point1 = point0 + offset
            point2 = point0 - offset
            cv2.line(img, tuple(point1.astype(int)), tuple(point2.astype(int)), color, thickness)

    def draw(self, img, color=(0, 0, 0), thickness=2):
        lines = np.concatenate((self.longitudinal_lines, self.latitudinal_lines), axis=0)
        self._draw_lines(lines, img, color, thickness)

def process(img):
    grid = Grid()
    grid.detect(img)
    grid.draw(img)

if __name__ == '__main__':
    #unit test
    display.visualizer(TEST_IMAGES, process)