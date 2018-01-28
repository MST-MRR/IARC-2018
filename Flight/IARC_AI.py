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

logging.basicConfig()

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

    def __init__(self):
        self._tower = Tower()
        self._tower.initialize()

        self._hovering = True
        self._lock = threading.Lock()
        self._last_image_retrieved = None
        self._reset_ai = False
        self._takeoff_lock = threading.Lock()
        self._controller = XboxOneController()
        self._x_vel = 0
        self._y_vel = 0
        self._z_vel = 0
        self._has_taken_off = False

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

        if self._controller.get_button_state(XboxOneController.A_BUTTON_ID) and not self._has_taken_off:
            with self._takeoff_lock:
                self._takeoff()
                self._has_taken_off = True
        if self._controller.get_button_state(XboxOneController.B_BUTTON_ID) and self._has_taken_off:
            with self._takeoff_lock:
                self._tower.land()
                self._has_taken_off = False

        if self._has_taken_off:
            if self._controller.get_button_state(XboxOneController.X_BUTTON_ID):
                self._hovering = True
            
            self._x_vel = self._controller.get_axis_state(XboxOneController.LEFT_JOYSTICK_HORIZONTAL_AXIS_ID) * SimpleDroneAI.CRUISING_SPEED
            self._y_vel = self._controller.get_axis_state(XboxOneController.LEFT_JOYSTICK_VERTICAL_AXIS_ID) * SimpleDroneAI.CRUISING_SPEED
            
            left_trigger_value = self._controller.get_axis_state(XboxOneController.LEFT_TRIGGER_AXIS_ID)
            right_trigger_value = self._controller.get_axis_state(XboxOneController.RIGHT_TRIGGER_AXIS_ID)

            if left_trigger_value > .1:
                self._z_vel = -left_trigger_value * SimpleDroneAI.CRUISING_SPEED
            elif right_trigger_value > .1:
                self._z_vel = right_trigger_value * SimpleDroneAI.CRUISING_SPEED
            else:
                self._z_vel = 0

            self._tower.fly(FlightVector(self._x_vel, self._y_vel, self._z_vel))

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
            
            print('Reset command receieved. Resetting AI...')
            self._tower.land()
            self._tower.disarm_drone()
            self._tower.shutdown()
            
            message = pygazebo.msg.gz_string_pb2.GzString()
            message.data = 'ack'
            
            yield From(publisher.publish(message))
            yield From(trollius.sleep(1))
            raise exc_type, value, traceback
        finally:
            cv2.destroyAllWindows()

while True:
    ardupilot_connection = ArdupilotGazeboMonitor()

    try:
        while not ardupilot_connection.ready_to_fly():
            time.sleep(1)
        
        ai = SimpleDroneAI()
        loop = trollius.get_event_loop()
        loop.run_until_complete(ai.run())
    except ResetSimulatorException:
        pass
    except KeyboardInterrupt:
        break
    finally:
        ardupilot_connection.stop()
