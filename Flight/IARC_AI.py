#!/usr/bin/env python2.7
############################################
# Multirotor Robot Design Team
# Missouri University of Science and Technology
# Spring 2018
# Christopher O'Toole

from AutonomousFlight import FlightVector
from ATC import Tower
import ATC
import cv2
import numpy as np
import time
import sys
import threading
import multiprocessing
import pygame
from sklearn.preprocessing import normalize

import pygazebo
import pygazebo.msg.image_stamped_pb2
import logging

import trollius
from trollius import From
from timeit import default_timer as timer
from xbox_one_controller import XboxOneController
from ardupilot_gazebo_monitor import ArdupilotGazeboMonitor
from ardupilot_gazebo_monitor import ArdupilotDisconnect

logging.basicConfig()

DEFAULT_TIMEOUT = 1

# displays a live stream of footage from the drone in the simulator, set to False for slightly better runtime performance
_DEBUG = True

class ResetSimulatorException(Exception):
    pass

class SimpleDroneAI():
    # reset event location
    RESET_EVENT_LOCATION = '/gazebo/default/reset'
    # reset event type
    RESET_EVENT_TYPE = 'gazebo.msgs.GzString'
    # camage image message location
    CAMERA_MSG_LOCATION = '/gazebo/default/realsense_camera/rs/stream/color'
    # camera message type
    CAMERA_MSG_TYPE = 'gazebo.msgs.ImageStamped'
    # altitude to hover at after takeoff in meters
    TAKEOFF_HEIGHT = 1.5
    # max speed (m/s)
    CRUISING_SPEED = 1

    def __init__(self, ardupilot_connection):
        self._tower = Tower()
        self._tower.initialize()
        self._ardupilot_connection = ardupilot_connection

        self._hovering = True
        self._lock = threading.Lock()
        self._last_image_retrieved = None
        self._reset_ai = False

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

        self._ardupilot_connection.update()

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

    def _reset(self, data):
        self._reset_ai = True

    def _failsafe_drone_shutdown(self):
        self._shutdown_drone_task = threading.Thread(target=self._tower.land)
        self._shutdown_drone_task.start()
        self._shutdown_drone_task.join(timeout=DEFAULT_TIMEOUT*10)

        if not self._shutdown_drone_task.is_alive():
            self._tower.disarm_drone()
            self._tower.shutdown()
        else: 
            self._ardupilot_connection.stop()

    def _reset_complete(self, publisher):
        print('Reset command receieved. Resetting AI...')
        self._failsafe_drone_shutdown()

    @trollius.coroutine
    def run(self):
        """
        @purpose: 
            Gazebo simulator event loop, forwards messages and state information to self._update
        @args:
        @returns: 
        """

        manager = yield From(pygazebo.connect()) 
        publisher = yield From(manager.advertise('/gazebo/default/reset_complete', 'gazebo.msgs.GzString'))

        subscriber = manager.subscribe(SimpleDroneAI.CAMERA_MSG_LOCATION, SimpleDroneAI.CAMERA_MSG_TYPE, self._event_handler)
        reset_event = manager.subscribe(SimpleDroneAI.RESET_EVENT_LOCATION, SimpleDroneAI.RESET_EVENT_TYPE, self._reset)

        try:
            yield From(subscriber.wait_for_connection())
            self._takeoff()
            # self._tower.fly(FlightVector(.5, 0, 0))

            while True:
                if self._reset_ai:
                    raise ResetSimulatorException()
                if self._last_image_retrieved is not None and self._lock.acquire(False):
                    try:
                        self._update(self._last_image_retrieved)
                    finally:
                        self._last_image_retrieved = None
                        self._lock.release()

                yield From(trollius.sleep(0.01))
        except ResetSimulatorException:
            exc_type, value, traceback = sys.exc_info()

            self._reset_complete(publisher)

            message = pygazebo.msg.gz_string_pb2.GzString()
            message.data = 'ack'
            
            yield From(publisher.publish(message))
            yield From(trollius.sleep(1))

            raise exc_type, value, traceback    
        finally:
            cv2.destroyAllWindows()

try:
    ardupilot_connection = ArdupilotGazeboMonitor(speedup=3)

    while True:
        try:
            if not ardupilot_connection.is_alive():
                ardupilot_connection.restart()
            
            while not ardupilot_connection.ready_to_fly():
                time.sleep(1)
            
            ai = SimpleDroneAI(ardupilot_connection)
            loop = trollius.get_event_loop()
            loop.run_until_complete(ai.run())
        except ResetSimulatorException:
            if ardupilot_connection.wait_for_gps_glitch(timeout=3):
                ardupilot_connection.reset()
        except ArdupilotDisconnect:
            ardupilot_connection.restart()
        except KeyboardInterrupt:
            break
finally:
    ardupilot_connection.stop()