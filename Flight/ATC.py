############################################
# This file contains a wrapper class
# for DroneKit related operations
# for our drone.
############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science Technology
# IARC 2018
# pylint: disable=C, F, I, R, W

from datetime import datetime, timedelta
from os import system
from time import sleep
from copy import deepcopy
from sys import stdout
from SerialSync import serialSync as gimbal
from marshmallow import Schema, fields, pprint
from AutonomousFlight import PIDFlightController

import dronekit
import math
import os
import time
import threading
import numpy as np

class VehicleStates(object):
  hover = "HOVER"
  flying = "FLYING"
  takeoff = "TAKEOFF"
  landing = "LANDING"
  landed = "LANDED"

class Tower(object):
  SIM = "tcp:127.0.0.1:5762"
  USB = "/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
  UDP = "192.168.12.1:14550"
  MAC = "/dev/cu.usbmodem1"
  LAND_ALTITUDE = 0.25
  ALT_PID_THRESHOLD = 0.21
  VEL_PID_THRESHOLD = 0.15
  VEL_PID_THRESHOLD_THROTTLE = 0.11
  BATTERY_FAILSAFE_VOLTAGE_SENTINEL = 13.25

  ASCEND_VELOCITY = np.array([0., 0., .3])

  def __init__(self, in_simulator=True):
    self.last_gimbal_angle = 105
    self.connection_str = self.SIM if in_simulator else self.USB
    self.state = VehicleStates.landed
    self.stop = threading.Event()
    self.takeoff_completed = threading.Event()
    self.land_completed = threading.Event()
    self.failsafe_controller = None
    self.connected = threading.Event()
    self.ready_for_serialization = threading.Event()
    self.connect()

  def _get_fields(self):
    return {
      'altitude': self.altitude,
      'airspeed': self.vehicle.airspeed,
      'velocity_x': self.vehicle.velocity[0],
      'velocity_y': self.vehicle.velocity[1],
      'velocity_z': self.vehicle.velocity[2],
      'voltage': self.vehicle.battery.voltage,
      'state': self.state,
      'mode': self.vehicle.mode.name,
      'armed': self.vehicle.armed,
      'roll': self.vehicle.attitude.roll,
      'pitch': self.vehicle.attitude.pitch,
      'yaw': self.vehicle.attitude.yaw,
      'altitude_controller_output': self.pid_flight_controller.altitude_pid.output,
      'altitude_rc_output': self.pid_flight_controller.altitude_pwm,
      'target_altitude': self.pid_flight_controller.target_altitude,
      'pitch_controller_output': self.pid_flight_controller.pitch_pid.output,
      'pitch_rc_output': self.pid_flight_controller.pitch_pwm,
      'target_pitch_velocity': self.pid_flight_controller.pitch_pid.SetPoint,
      'roll_controller_output': self.pid_flight_controller.roll_pid.output,
      'roll_rc_output': self.pid_flight_controller.roll_pwm,
      'target_roll_velocity': self.pid_flight_controller.roll_pid.SetPoint,
      'yaw_controller_output': self.pid_flight_controller.yaw_pid.output,
      'yaw_rc_output': self.pid_flight_controller.yaw_pwm,
      'target_yaw': self.pid_flight_controller.yaw_pid.SetPoint
    }
  
  @property
  def json(self):
    ignore=('mode', 'altitude', 'state')

    for name, value in self._get_fields().items():
      if name not in ignore:
        setattr(self, name, value)
    
    return self.schema.dumps(self)

  def connect(self):
    def attempt_to_connect():
      connected = False
      while not connected and not self.stop.is_set():
        try:
          self.vehicle = dronekit.connect(self.connection_str, wait_ready=True)
        except:
          pass
        else:
          if not self.stop.is_set():
            print('Connected to vehicle.')
            self.connected.set()
            connected = True

            self.pid_flight_controller = PIDFlightController(self)
            self.pid_flight_controller.write_to_rc_channels(should_flush_channels=True)
            self.switch_control()

            self.failsafe_controller = FailsafeController(self)
            self.failsafe_controller.daemon = True
            self.failsafe_controller.start()
  
            def to_marshmellow_field(v):
              print(v)
              cast = {str: fields.Str(), float: fields.Float(), int: fields.Integer(), bool: fields.Boolean()}
              return cast[type(v)]

            self.schema = type('ATC_Schema', (Schema,), {k: to_marshmellow_field(v) for k, v in self._get_fields().items()})()
            self.ready_for_serialization.set()
            
    
    connection_thread = threading.Thread(target=attempt_to_connect)
    connection_thread.daemon = True
    connection_thread.start()

  def _assert_vehicle_is_connected(self):
    assert self.connected.is_set(), 'Not yet connected to vehicle'

  def shutdown(self):
    self._assert_vehicle_is_connected()
    self.stop.set()
    if self.failsafe_controller is not None:
      self.failsafe_controller.stop.set()
    self.vehicle.close()
    self.connected.clear()
    self.takeoff_completed.clear()
    self.land_completed.clear()

  def arm_drone(self):
    self._assert_vehicle_is_connected()
    self.vehicle.armed = True
  
  def send_gimbal_message(self):
        gimbal.send(105 + int(math.degrees(self.vehicle.attitude.pitch)))
    self.last_gimbal_angle = 105 + int(math.degrees(self.vehicle.attitude.pitch))

  @property
  def is_armed(self):
    return self.vehicle.armed
  
  def disarm_drone(self):
    self._assert_vehicle_is_connected()
    self.vehicle.armed = False

  @property
  def is_disarmed(self):
    return not self.vehicle.armed

  @property
  def mode(self):
    return self.vehicle.mode.name

  def switch_control(self, target_mode="LOITER"):
    self._assert_vehicle_is_connected()
    self.target_mode = dronekit.VehicleMode(target_mode)

    if self.mode != target_mode:
      self.vehicle.mode = self.target_mode
  
  @property
  def reached_target_control_mode(self):
    return self.target_mode == self.vehicle.mode

  @property
  def altitude(self):
    self._assert_vehicle_is_connected()
    return self.vehicle.rangefinder.distance

  def in_range(self, threshold, base_value, num):
    return np.abs(base_value-num) <= threshold

  def takeoff(self, target_altitude):
    self.land_completed.clear()
    self.switch_control()
    while not self.reached_target_control_mode: time.sleep(.001)
    self.arm_drone()

    self.state = VehicleStates.takeoff
    self.pid_flight_controller.target_velocity = Tower.ASCEND_VELOCITY
    self.pid_flight_controller.target_altitude = target_altitude
  
    def hover_after_takeoff():
      while not self.in_range(self.ALT_PID_THRESHOLD, self.altitude, target_altitude):
        time.sleep(.001)

      self.takeoff_completed.set()
      self.hover()

    takeoff_complete_event_thread = threading.Thread(target=hover_after_takeoff)
    takeoff_complete_event_thread.daemon = True
    takeoff_complete_event_thread.start()

  def fly(self, velocity):
    assert self.takeoff_completed.is_set(), 'Cannot fly until after takeoff.'
    self.state = VehicleStates.flying
    self.pid_flight_controller.target_velocity = velocity
    
  def hover(self):
    assert self.takeoff_completed.is_set(), 'Cannot hover until after takeoff'
    self.pid_flight_controller.target_velocity = np.array([0., 0., 0.])
    self.state = VehicleStates.hover
  
  def land(self):
    assert self.takeoff_completed.is_set(), 'Cannot land until after takeoff'
    self.hover()
    self.switch_control(target_mode='LAND')
    self.state = VehicleStates.landing    
    self.pid_flight_controller.write_to_rc_channels(should_flush_channels=True)

    def handle_landing_complete():
      while not self.in_range(self.ALT_PID_THRESHOLD, self.altitude, self.LAND_ALTITUDE):
        time.sleep(.001)

      self.state = VehicleStates.landed
      self.land_completed.set()
      self.takeoff_completed.clear()

    landing_complete_event_thread = threading.Thread(target=handle_landing_complete)
    landing_complete_event_thread.daemon = True
    landing_complete_event_thread.start()

class FailsafeController(threading.Thread):
  def __init__(self, atc_instance):
    self.atc = atc_instance
    self.stop = threading.Event()
    super(FailsafeController, self).__init__()

  def run(self):
    while not self.stop.is_set():
      self.atc.pid_flight_controller.update_controllers()
      # print self.atc.last_gimbal_angle, int(math.degrees(self.atc.vehicle.attitude.pitch))
      if (105 + int(math.degrees(self.atc.vehicle.attitude.pitch))) != self.atc.last_gimbal_angle:
          self.atc.send_gimbal_message()
      if self.atc.vehicle.armed and self.atc.vehicle.mode.name == "LOITER":
        self.atc.pid_flight_controller.write_to_rc_channels()
        
    
      sleep(.01)
