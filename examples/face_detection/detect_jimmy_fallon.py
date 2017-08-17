############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

import sys
import os
sys.path.append('../..')

import mrrdt_vision
import cv2
import numpy as np

JIMMY_FALLON_VIDEO = 'jimmy.mp4'
QUIT_KEY = 'q'

# bounding box color
GREEN = (0, 255, 0)
# bounding box thickness
THICKNESS = 3

class VideoReader():
  def __init__(self, fileName):
    self.fileName = fileName

  def __enter__(self):
    self.cap = cv2.VideoCapture(self.fileName)
    return self

  def next(self):
    if (not self.cap.isOpened()):
      return None

    return self.cap.read()[1]

  def __exit__(self, *args):
    self.cap.release()

with VideoReader(JIMMY_FALLON_VIDEO) as video:
  frame = video.next()
  should_quit = False

  while frame is not None and not should_quit:
    frame = cv2.resize(frame, (640, 480))
    detections = mrrdt_vision.detect_object(frame, mrrdt_vision.ObjectTypes.FACE, num_pyramid_levels=1)

    for (x_min, y_min, x_max, y_max) in detections: 
      cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), GREEN, THICKNESS)

    cv2.imshow(JIMMY_FALLON_VIDEO, frame)
    should_quit = (chr(cv2.waitKey(1) & 0xFF) == QUIT_KEY)
    frame = video.next()