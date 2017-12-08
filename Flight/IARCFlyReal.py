#!/usr/bin/env python2.7
############################################
# Multirotor Robot Design Team
# Missouri University of Science and Technology
# Fall 2017
# Christopher O'Toole

'''
A simple quadcopter AI which follows the nearest roomba in an IARC arena simulated by Gazebo
'''

from AutonomousFlight import FlightVector
from ATC import Tower
import cv2
import numpy as np
import time
from sklearn.preprocessing import normalize

import pygazebo
import pygazebo.msg.image_stamped_pb2
import logging

import trollius
from trollius import From
from timeit import default_timer as timer

import mrrdt_vision
from mrrdt_vision.obj_detect.roomba_cnn import RoombaDetector

import pyrealsense as pyrs

logging.basicConfig()

# displays a live stream of footage from the drone in the simulator, set to False for slightly better runtime performance
_DEBUG = False

class SimpleDroneAI():
    # camage image message location
    CAMERA_MSG_LOCATION = '/gazebo/default/iris/iris_demo/gimbal_small_2d/tilt_link/camera/image'
    # camera message type
    CAMERA_MSG_TYPE = 'gazebo.msgs.ImageStamped'
    # speed to follow roombas with in m/s
    ROOMBA_TRACKING_SPEED = .4
    # altitude to hover at after takeoff in meters
    TAKEOFF_HEIGHT = 1.5
    # time in seconds we can go without finding a roomba before going into hover mode
    MAX_LOST_TARGET_TIME = 3

    def __init__(self):
        self._tower = Tower()
        self._tower.initialize()
        self._detector = RoombaDetector()
        self._time_since_last_roomba = 0
        self._hovering = True

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

    def _get_velocity_vector2d(self, start, goal, speed):
        """
        @purpose: 
            Takeoff and hover at an altitude of `SimpleDroneAI.TAKEOFF_HEIGHT` meters
            Note: This function will dampen the speed based on the distance we are away from the target
        @args:
            start, np.array: 2D position vector corresponding to the start point
            goal, np.array: 2D position vector corresponding to the destination point
            speed, float: speed limit in m/s
        @returns: 
            A FlightVector pointed in the direction of `goal` with speed less than or equal to `speed`.
        """
        dist = np.sqrt(np.sum((goal-start)**2))
        x_vel, y_vel = min(dist, SimpleDroneAI.ROOMBA_TRACKING_SPEED)*normalize((goal - start).reshape(-1, 1))
        return FlightVector(-y_vel, x_vel, 0)

    def _follow_nearest_roomba(self, roombas, drone_midpoint):
        """
        @purpose: 
            Attempts to follow the roomba closest to the drone
        @args:
            roombas, collection of Roomba instances: A list of detected roombas
            drone_midpoint, np.array: Midpoint of the drone, we navigate relative to this point
        @returns: 
        """
        if roombas:
            roomba_midpoints = np.asarray([roomba.center for roomba in roombas])
            target_idx = np.argmin(np.sum((roomba_midpoints-drone_midpoint)**2, axis=1))
            target = roombas[target_idx]

            velocity_vector = self._get_velocity_vector2d(drone_midpoint, roomba_midpoints[target_idx], SimpleDroneAI.ROOMBA_TRACKING_SPEED)
            self._tower.fly(velocity_vector)
            self.hovering = False
            self._time_since_last_roomba = timer()
        elif timer() - self._time_since_last_roomba >= SimpleDroneAI.MAX_LOST_TARGET_TIME and not self.hovering:
            self.hovering = True

    def _update(self, data):
        """
        @purpose: 
            Event handler which updates relevant state variables and issues commands to the drone
        @args:
            data, protobuf: A message containing an image from the drone's camera.
        @returns: 
        """
        message = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
        h, w = (message.image.height, message.image.width)
        img = np.fromstring(message.image.data, dtype=np.uint8).reshape(h, w, 3)
        roombas = self._detector.detect(img)
        
        drone_midpoint = np.asarray([w/2, h/2])
        self._follow_nearest_roomba(roombas, drone_midpoint)

        if _DEBUG:
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            for roomba in roombas:
                roomba.draw(bgr_img)
            
            cv2.imshow('debug', bgr_img)
            cv2.waitKey(1)

    @trollius.coroutine
    def run(self):
        """
        @purpose: 
            Gazebo simulator event loop, forwards messages and state information to self._update
        @args:
        @returns: 
        """
        self._takeoff()
        
        ## start the service - also available as context manager
        serv = pyrs.Service()

        ## create a device from device id and streams of interest
        cam = serv.Device(device_id = 0, streams = [pyrs.stream.ColorStream(fps = 60)])
        
        cam.wait_for_frames()
        self.update(cam.color)
        
        cam.stop()
        serv.stop()
        

        while True:
            yield From(trollius.sleep(1.00))

ai = SimpleDroneAI()
loop = trollius.get_event_loop()
loop.run_until_complete(ai.run())
