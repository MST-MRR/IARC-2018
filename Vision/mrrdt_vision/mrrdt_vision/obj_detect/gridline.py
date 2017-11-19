#!/usr/bin/env python3.5
############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Fall 2017
# Christopher O'Toole

import os
import cv2
import numpy as np
import threshold_gpu
from collections import defaultdict

# path to folder containing test images
DATA_FOLDER_PATH = '../data/'
# keyword which indicates the test image corresponds to this file
IMG_NAME_KEYWORD = 'Roblox'
# key to stop the program 
QUIT_KEY = 'q'

# line confidence threshold
LINE_THRESHOLD = 150
# lower canny parameter
LOWER_CANNY = 150
# upper canny parameter
UPPER_CANNY = 200
# median blur kernel size
MEDIAN_BLUR_KERNEL_SIZE = 5
# line display color
LINE_COLOR = (255, 255, 0)
# line display thickness
LINE_THICKNESS = 3
# mulitplier for extending drawn lines
LINE_MULTIPLIER = 100000
# small constant to prevent division by zero errors
EPSILON = 1e-10

def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]


def segmented_intersections(lines):
    """Finds the intersections between groups of lines."""

    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersections.append(intersection(line1, line2)) 

    return intersections

def segment_by_angle_kmeans(lines, k=2, **kwargs):
    """Groups lines based on angle with k-means.

    Uses k-means on the coordinates of the angle on the unit circle 
    to segment `k` angles inside `lines`.
    """

    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    attempts = kwargs.get('attempts', 10)
    criteria = kwargs.get('criteria', (default_criteria_type, attempts, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_RANDOM_CENTERS)

    # returns angles in [0, pi] in radians
    angles = np.array([line[0][1] for line in lines])
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented

def order_points(pts):
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype = "float32")
 
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
 
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
 
    # return the ordered coordinates
    return rect

def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
 
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
 
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
 
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
 
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
 
    # return the warped image
    return warped

def skeletonize(img):
    """ OpenCV function to return a skeletonized version of img, a Mat object"""

    #  hat tip to http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/

    img = img.copy() # don't clobber original
    skel = img.copy()

    skel[:,:] = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

    while True:
        eroded = cv2.morphologyEx(img, cv2.MORPH_ERODE, kernel)
        temp = cv2.morphologyEx(eroded, cv2.MORPH_DILATE, kernel)
        temp  = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img[:,:] = eroded[:,:]
        if cv2.countNonZero(img) == 0:
            break

    return skel

def find_closest_point_to(point, points):
    return points[np.argmin(np.sum((points-point)**2, axis = 1))]

class Grid:
    def __init__(self):
        self._lines = None

    @property
    def lines(self):
        return self._lines

    @property
    def intersections(self):
        segmented = segment_by_angle_kmeans(self.lines)
        intersections = np.asarray(segmented_intersections(segmented))
        return intersections[:, 0, :] if len(intersections) > 0 else None

    def detect_gridlines(self, img):
        blur = cv2.GaussianBlur(img, (15, 15), 3.38)
        edges = cv2.Canny(cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY), 85, 30)
        dilation = cv2.dilate(edges, np.ones((7,7),np.uint8), iterations=3)

        lines = cv2.HoughLines(skeletonize(dilation), 1.0, np.pi/180, LINE_THRESHOLD)

        if lines is not None:
            self._lines = lines[:, 0, :]
        
        self._lines = lines

    def draw_line(self, img, line, color=LINE_COLOR, thickness=LINE_THICKNESS):
        rho, theta = line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 - LINE_MULTIPLIER*b)
        y1 = int(y0 + LINE_MULTIPLIER*a)
        x2 = int(x0 + LINE_MULTIPLIER*b)
        y2 = int(y0 - LINE_MULTIPLIER*a)
        cv2.line(img, (x1,y1), (x2,y2), color, thickness)
    
    def draw(self, img):
        if self._lines is not None:
            for line in self._lines[:, 0, :]:
                self.draw_line(img, line)


if __name__ == '__main__':
    pass