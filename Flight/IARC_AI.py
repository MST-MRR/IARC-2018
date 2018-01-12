#!/usr/bin/env python2.7
############################################
# Multirotor Robot Design Team
# Missouri University of Science and Technology
# Spring 2018
# Christopher O'Toole

from AutonomousFlight import FlightVector
from ATC import Tower
import cv2
import numpy as np
import time
import threading
from sklearn.preprocessing import normalize

import pygazebo
import pygazebo.msg.image_stamped_pb2
import logging

import trollius
from trollius import From
from timeit import default_timer as timer

logging.basicConfig()

# displays a live stream of footage from the drone in the simulator, set to False for slightly better runtime performance
_DEBUG = True

class SimpleDroneAI():
    # camage image message location
    CAMERA_MSG_LOCATION = '/gazebo/default/realsense_camera/rs/stream/color'
    # camera message type
    CAMERA_MSG_TYPE = 'gazebo.msgs.ImageStamped'
    # altitude to hover at after takeoff in meters
    TAKEOFF_HEIGHT = 1.5

    def __init__(self):
        self._tower = Tower()
        self._tower.initialize()

        self._hovering = True
        self._lock = threading.Lock()
        self._last_image_retrieved = None

    @property
    def hovering(self):
        return self._hovering

    @hovering.setter
    def hovering(self, value):
        self._hovering = value

        if self.hovering:
            self._tower.hover()

    def _takeoff(self):
        """
        @purpose: Takeoff and hover at an altitude of `SimpleDroneAI.TAKEOFF_HEIGHT` meters
        @args:
        @returns:
        """
        self._tower.takeoff(SimpleDroneAI.TAKEOFF_HEIGHT)
    
    def _update(self, img):
        """
        @purpose: 
            Event handler which updates relevant state variables and issues commands to the drone
        @args:
            img, np.ndarray: An image from the drone's camera.
        @returns: 
        """

        if _DEBUG:
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow('debug', bgr_img)
            cv2.waitKey(1)

    def _event_handler(self, data):
        """
        @purpose: 
            Event handler which updates relevant state variables
        @args:
            data, protobuf: A message containing an image from the drone's camera.
        @returns: 
        """
        message = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
        h, w = (message.image.height, message.image.width)
        img = np.fromstring(message.image.data, dtype=np.uint8).reshape(h, w, 3)

        with self._lock:
            self._last_image_retrieved = img

    @trollius.coroutine
    def run(self):
        """
        @purpose: 
            Gazebo simulator event loop, forwards messages and state information to self._update
        @args:
        @returns: 
        """
        self._takeoff()
        manager = yield From(pygazebo.connect()) 
        
        subscriber = manager.subscribe(SimpleDroneAI.CAMERA_MSG_LOCATION, SimpleDroneAI.CAMERA_MSG_TYPE, self._event_handler)
        yield From(subscriber.wait_for_connection())

        self._tower.fly(FlightVector(.33, 0, 0))

        while True:
            if self._last_image_retrieved is not None and self._lock.acquire(False):
                try:
                    self._update(self._last_image_retrieved)
                finally:
                    self._last_image_retrieved = None
                    self._lock.release()

            yield From(trollius.sleep(0.01))

ai = SimpleDroneAI()
loop = trollius.get_event_loop()
loop.run_until_complete(ai.run())