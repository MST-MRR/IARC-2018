#!/usr/bin/env python2.7
############################################
# Multirotor Robot Design Team
# Missouri University of Science and Technology
# Spring 2018
# Christopher O'Toole

import math
from ATC import Tower, VehicleStates
import ATC
import cv2
import numpy as np
import time
import sys
import threading
import multiprocessing
from mrrdt_vision.obj_detect.roomba_cnn import RoombaDetector
from timeit import default_timer as timer
from sklearn.preprocessing import normalize
from Client import send_message

import pygazebo
import pygazebo.msg.image_stamped_pb2
import logging
import argparse

import trollius
from trollius import From
from timeit import default_timer as timer
from ardupilot_gazebo_monitor import ArdupilotGazeboMonitor, QuadcopterCrash, READY_TO_FLY_TIMEOUT

logging.basicConfig()

DEFAULT_TIMEOUT = .001

# displays a live stream of footage from the drone in the simulator, set to False for slightly better runtime performance
_DEBUG = True

class ResetSimulatorException(Exception):
    pass

class ArdupilotDisconnect(Exception):
    pass

class SimpleDroneAI():
    # reset event location
    RESET_EVENT_LOCATION = '/gazebo/default/reset'
    # gazebo reset complete location
    RESET_COMPLETE_EVENT_LOCATION = '/gazebo/default/reset_complete'
    # reset event type
    RESET_EVENT_TYPE = 'gazebo.msgs.GzString'
    # color camage image message location
    COLOR_CAMERA_MSG_LOCATION = '/gazebo/default/realsense_camera/rs/stream/color'
    # depth camera image message location
    DEPTH_CAMERA_MSG_LOCATION = '/gazebo/default/realsense_camera/rs/stream/depth'
    # depth far clip
    DEPTH_FAR_CLIP_MM = 10000
    # depth near clip
    DEPTH_NEAR_CLIP_MM = 300
    # camera fov
    CAMERA_FIELD_OF_VIEW = 60
    # color image stream window title
    COLOR_IMAGE_STREAM_DEBUG_NAME = 'color'
    # depth image stream window title
    DEPTH_IMAGE_STREAM_DEBUG_NAME = 'depth'
    # camera message type
    CAMERA_MSG_TYPE = 'gazebo.msgs.ImageStamped'
    # altitude to hover at after takeoff in meters
    TAKEOFF_HEIGHT = 2
    # maximum time spent on drone shutdown task
    DRONE_SHUTDOWN_TASK_TIMEOUT = 7
    # max speed (m/s)
    CRUISING_SPEED = 1
    # roomba speed (m/s)
    ROOMBA_SPEED = .33
    # speed to follow roombas at normally
    ROOMBA_TRACKING_SPEED = .33
    # hover if drone goes beyond this speed
    DRONE_HARD_SPEED_LIMIT = 1.2*ROOMBA_TRACKING_SPEED
    # stop when we are this many meters away from the roomba
    ROOMBA_DISTANCE_THRESHOLD = .1
    # hover after losing the roomba for this much time in seconds
    MAX_LOST_TARGET_TIME = 10

    def __init__(self, ardupilot_connection):
        self._tower = Tower()
        self._ardupilot_connection = ardupilot_connection

        self._lock = threading.Lock()
        self._depth_image_lock = threading.Lock()
        self._last_image_retrieved = None
        self._last_depth_image_retrieved = None
        self._roomba_detector = RoombaDetector(threshold=.7)
        self._reset_ai = False
        self._time_since_last_roomba = 0

    def get_altitude(self):
        return self._tower.altitude

    def is_drone_above_speed_limit(self):
        return self._tower.vehicle.airspeed > self.DRONE_HARD_SPEED_LIMIT
    
    def get_meters_per_pixel(self, img):
        image_width = img.shape[1]
        image_width_in_meters = 2*math.tan(math.radians(self.CAMERA_FIELD_OF_VIEW/2.)) * self.get_altitude()
        return image_width_in_meters / image_width

    def get_velocity_vector2d(self, img, start, goal, speed):
        dist = float(np.sqrt(np.sum((goal-start)**2)))*self.get_meters_per_pixel(img)
        x_vel, y_vel = min(dist if dist > self.ROOMBA_DISTANCE_THRESHOLD else 0, speed)*normalize((goal-start).reshape(-1, 1), axis=1)
        return np.array([-y_vel, x_vel, 0])

    def _update(self, img, depth_img):
        roombas = None

        if self._tower.takeoff_completed.is_set():
            h, w = img.shape[:2]
            drone_midpoint = np.asarray([w/2, h/2])
            roombas = self._roomba_detector.detect(img)

            if self.is_drone_above_speed_limit():
               self._tower.hover()
            
            if roombas:
                roomba_midpoints = np.asarray([roomba.center for roomba in roombas])
                target_idx = np.argmin(np.sum((roomba_midpoints-drone_midpoint)**2, axis=1))
                target = roombas[target_idx]
                velocity = self.get_velocity_vector2d(img, drone_midpoint, roomba_midpoints[target_idx], self.ROOMBA_TRACKING_SPEED)
                self._tower.fly(velocity)
                self._time_since_last_roomba = timer()
            elif timer() - self._time_since_last_roomba >= self.MAX_LOST_TARGET_TIME:
                self._tower.hover()

        if _DEBUG:
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            if roombas is not None:
                for roomba in roombas:
                    roomba.draw(bgr_img)
            
            norm = (depth_img.astype(np.float32)-depth_img.min())/(depth_img.max()-depth_img.min()+1e-9)*255
            gray = norm.astype(np.uint8)
            color = cv2.applyColorMap(gray, cv2.COLORMAP_AUTUMN)

            if self._tower.ready_for_serialization.is_set():
                self._tower.color_image = bgr_img
                self._tower.depth_image = color
                send_message(self._tower.json)

        return self._ardupilot_connection.update()

    def _depth_image_handler(self, data):
        message = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
        h, w = (message.image.height, message.image.width)
        img = np.fromstring(message.image.data, dtype=np.uint16).reshape(h, w)

        with self._depth_image_lock:
            self._last_depth_image_retrieved = img

    def _event_handler(self, data):
        message = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
        h, w = (message.image.height, message.image.width)
        img = np.fromstring(message.image.data, dtype=np.uint8).reshape(h, w, 3)

        with self._lock:
            self._last_image_retrieved = img

    def _reset(self, data):
        self._reset_ai = True

    def _failsafe_drone_shutdown(self):
        tasks = []

        if self._tower.state != VehicleStates.landed:
            for task in [self._tower.land, self._tower.disarm_drone, self._tower.shutdown]:
                tasks.append(threading.Thread(target=task))
                tasks[-1].start()
                tasks[-1].join(timeout=SimpleDroneAI.DRONE_SHUTDOWN_TASK_TIMEOUT)
                if task == self._tower.land:
                    self._tower.land_completed.wait(timeout=SimpleDroneAI.DRONE_SHUTDOWN_TASK_TIMEOUT)

            if any([task.is_alive() for task in tasks]):
                ardupilot_connection.stop(force=True)

    def _reset_complete(self, publisher):
        print('Reset command receieved. Resetting AI...')
        self._failsafe_drone_shutdown()

    @trollius.coroutine
    def run(self):
        manager = yield From(pygazebo.connect()) 
        publisher = yield From(manager.advertise(SimpleDroneAI.RESET_COMPLETE_EVENT_LOCATION, SimpleDroneAI.RESET_EVENT_TYPE))

        subscriber = manager.subscribe(SimpleDroneAI.COLOR_CAMERA_MSG_LOCATION, SimpleDroneAI.CAMERA_MSG_TYPE, self._event_handler)
        depth_image_subscriber = manager.subscribe(SimpleDroneAI.DEPTH_CAMERA_MSG_LOCATION, SimpleDroneAI.CAMERA_MSG_TYPE, self._depth_image_handler)
        reset_event = manager.subscribe(SimpleDroneAI.RESET_EVENT_LOCATION, SimpleDroneAI.RESET_EVENT_TYPE, self._reset)

        try:
            yield From(subscriber.wait_for_connection())
            yield From(depth_image_subscriber.wait_for_connection())
            self._tower.connected.wait()
            self._tower.takeoff(SimpleDroneAI.TAKEOFF_HEIGHT)

            while True:
                if self._reset_ai:
                    raise ResetSimulatorException()
                if self._last_image_retrieved is not None and self._last_depth_image_retrieved is not None:
                    try:
                        with self._depth_image_lock as depth_image_lock, self._lock as lock:
                            if not self._update(self._last_image_retrieved, self._last_depth_image_retrieved):
                                raise ArdupilotDisconnect()
                    finally:
                        self._last_image_retrieved = None
                        self._last_depth_image_retrieved = None

                yield From(trollius.sleep(0.01))
        except (ResetSimulatorException, QuadcopterCrash, ArdupilotDisconnect) as e:
            if isinstance(e, QuadcopterCrash):
                print('Quadcopter crash detected! Interpreting as a cue to reset the simulator...')
        
            exc_type, value, traceback = sys.exc_info()

            self._reset_complete(publisher)

            message = pygazebo.msg.gz_string_pb2.GzString()
            message.data = 'crash' if isinstance(e, (QuadcopterCrash, ArdupilotDisconnect)) else 'ack' 
            
            yield From(publisher.publish(message))
            yield From(trollius.sleep(1))

            raise exc_type, value, traceback    
        finally:
            cv2.destroyAllWindows() 
            self._failsafe_drone_shutdown()

parser = argparse.ArgumentParser(description='Processes AI settings passed in')
parser.add_argument('--test', '-T', type=int, help='test experimental features', default=1)
args = parser.parse_args()

if not args.test:
    with ArdupilotGazeboMonitor(speedup=3) as ardupilot_connection:
        while True:
            try:
                stopping_condition = lambda: ardupilot_connection.is_ready() and ardupilot_connection.wait_for_ready_to_fly(timeout=DEFAULT_TIMEOUT)
                while not stopping_condition():
                    start = timer()

                    while ardupilot_connection.update() and (timer()-start) <= READY_TO_FLY_TIMEOUT:
                        if ardupilot_connection.wait_for_ready_to_fly(timeout=DEFAULT_TIMEOUT):
                            break
                
                    if not stopping_condition():
                        ardupilot_connection.restart()

                ai = SimpleDroneAI(ardupilot_connection)
                loop = trollius.get_event_loop()
                loop.run_until_complete(ai.run())
            except QuadcopterCrash:
                ardupilot_connection.flush()
                ardupilot_connection.reset()
                ardupilot_connection.restart()
            except ResetSimulatorException:
                if ardupilot_connection.wait_for_gps_glitch(timeout=3):
                    ardupilot_connection.reset()
            except ArdupilotDisconnect:
                ardupilot_connection.restart()
            except KeyboardInterrupt:
                break
else:
    def _depth_image_handler(data):
        message = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
        h, w = (message.image.height, message.image.width)
        img = np.fromstring(message.image.data, dtype=np.uint16).reshape(h, w)
        zeros = np.zeros((h, w), dtype=np.uint16)
        
        if _DEBUG:
            scale = (SimpleDroneAI.DEPTH_FAR_CLIP_MM-SimpleDroneAI.DEPTH_NEAR_CLIP_MM)
            offset = SimpleDroneAI.DEPTH_NEAR_CLIP_MM
            norm = np.maximum(img.astype(np.float32)-offset, zeros)/scale * 255
            gray = norm.astype(np.uint8)
            color = cv2.applyColorMap(gray, cv2.COLORMAP_AUTUMN)
            cv2.imshow('depth', color)
            cv2.waitKey(1)

    @trollius.coroutine
    def run():
        manager = yield From(pygazebo.connect())
        depth_image_subscriber = manager.subscribe(SimpleDroneAI.DEPTH_CAMERA_MSG_LOCATION, SimpleDroneAI.CAMERA_MSG_TYPE, _depth_image_handler)

        try:
            yield From(depth_image_subscriber.wait_for_connection())

            while True:
                yield From(trollius.sleep(0.01))
        except KeyboardInterrupt as e:
            pass
        finally:
            cv2.destroyAllWindows()

    loop = trollius.get_event_loop()
    loop.run_until_complete(run())