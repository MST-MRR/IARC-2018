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
import sys
from time import sleep
from copy import deepcopy


import dronekit
import math
import os
import time
import threading

class DroneAttitude():

  def __init__(self, roll, pitch, yaw):
    self.pitch_deg = pitch
    self.yaw_deg = yaw
    self.roll_deg = roll
    self.pitch = math.radians(pitch)
    self.yaw = math.radians(yaw)
    self.roll = math.radians(roll)
    self.quaternion = self.get_quaternion()

  def get_quaternion(self):
    t0 = math.cos(self.yaw * 0.5)
    t1 = math.sin(self.yaw * 0.5)
    t2 = math.cos(self.roll * 0.5)
    t3 = math.sin(self.roll * 0.5)
    t4 = math.cos(self.pitch * 0.5)
    t5 = math.sin(self.pitch * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

class StandardAttitudes(object):
  level = DroneAttitude(0,0,0)
  forward = DroneAttitude(0,-5,0)
  backward = DroneAttitude(0,5,0)
  left = DroneAttitude(-5, 0, 0)
  right = DroneAttitude(5, 0, 0)

class StandardThrusts(object):
  none = 0.00
  low = 0.25
  land = 0.25
  hover = 0.525
  takeoff = 0.75
  full = 1.00

class VehicleStates(object):
  hover = "HOVER"
  flying = "FLYING"
  takeoff = "TAKEOFF"
  unknown = "UNKNOWN"
  avoidance = "AVOIDANCE"
  landing = "LANDING"
  landed = "LANDED"

class Tower(object):
  SIM = "tcp:127.0.0.1:5760"
  USB = "/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
  UDP = "192.168.12.1:14550"
  MAC = "/dev/cu.usbmodem1"
  STANDARD_ATTITUDE_BIT_FLAGS = 0b00111111
  STANDARD_THRUST_CHANGE = 0.05
  MAX_TURN_TIME = 5
  LAND_ALTITUDE = 0.5
  TURN_START_VELOCITY = 3
  TURN_RADIUS = 0.5 # Meters
  STANDARD_ANGLE_ADJUSTMENT = 1.0
  MESSAGE_SLEEP_TIME = 0.01
  STANDARD_SLEEP_TIME = 1
  MAX_ANGLE_ALL_AXIS = 15.0
  BATTERY_FAILSAFE_VOLTAGE_PANIC = 9.25
  BATTERY_FAILSAFE_VOLTAGE_SENTINEL = 13.25

  def __init__(self):
    self.start_time = 0
    self.flight_log = None
    self.vehicle_initialized = False
    self.vehicle = None
    self.scanse = None
    self.LAST_ATTITUDE = StandardAttitudes.level
    self.LAST_THRUST = StandardThrusts.none
    self.STATE = VehicleStates.unknown

  def initialize(self, should_write_to_file=False):
    """
    @purpose: Connect to the flight controller, start the failsafe
              thread, switch to GUIDED_NOGPS, and open a file to
              begin logging.
    @args:
    @returns:
    """
    if(not self.vehicle_initialized):

      if(should_write_to_file):
        self.flight_log = open('flight_log.txt', 'w')
        sys.stdout = self.flight_log

      print("\nConnecting to vehicle...")
      self.vehicle = dronekit.connect(self.SIM, wait_ready=True)

      if not self.vehicle:
        print("\nUnable to connect to vehicle.")
        return

      self.vehicle.mode = dronekit.VehicleMode("STABILIZE")
      self.STATE = VehicleStates.landed
      self.vehicle_initialized = True
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
      self.start_time = int(time.time())

      self.switch_control()

      print("\nSuccessfully connected to vehicle.")

  def shutdown(self):
    """
    @purpose: Stop all operations and cleanup the vehicle object.
    @args:
    @returns:
    """
    self.failsafes.join()
    self.vehicle.close()
    if(self.flight_log):
      self.flight_log.close()
    self.vehicle_initialized = False
    self.start_time = 0

  def arm_drone(self):
    """
    @purpose: Arm the vehicle.
    @args:
    @returns:
    """
    self.vehicle.armed = True
    while(not self.vehicle.armed):
      sleep(self.STANDARD_SLEEP_TIME)

  def disarm_drone(self):
    """
    @purpose: Disarm the vehicle.
    @args:
    @returns:
    """
    self.vehicle.armed = False
    while(self.vehicle.armed):
      sleep(self.STANDARD_SLEEP_TIME)

  def switch_control(self, mode_name="STABILIZE"):
    """
    @purpose: Switch the mode to GUIDED_NOGPS and make sure
             that the failsafe thread is running.
    @args:
    @returns:
    """
    if not self.failsafes:
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
    if self.vehicle.mode.name != mode_name:
      self.vehicle.mode = dronekit.VehicleMode(mode_name)
      while(self.vehicle.mode.name != mode_name):
        sleep(self.STANDARD_SLEEP_TIME)

  def get_uptime(self):
    """
    @purpose: Get up time of this object.
    @args:
    @returns:
    """
    uptime = int(time.time()) - self.start_time
    return uptime

  def map(self, x, in_min, in_max, out_min, out_max):
    """
    @purpose: Re-maps a number from one range to another.
    @args:
      x: the number to map
      in_min: the lower bound of the value's current range
      in_max: the upper bound of the value's current range
      out_min: the lower bound of the value's target range
      out_max: the upper bound of the value's target range
    @returns:
      The mapped value.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

  def set_angle_thrust(self, attitude, thrust):
    """
    @purpose: Send a specified attitude message to the
              flight controller. For more information, see
              http://mavlink.org/messages/common#SET_ATTITUDE_TARGET.
    @args:
      attitude: A DroneAtittude object containing a target attitude.
      thrust: A collective thrust from 0 to 1. Thrust is converted to
              a climb rate internally by the flight controller. Therefore,
              thrusts from 0.51 to 1 are climb rates and thrusts from 0.49
              to 0 are descent rates. 0.50 attempts to maintain a hover.
    @returns:
    """
    while(self.vehicle.mode.name != "GUIDED_NOGPS"):
      sleep(self.STANDARD_SLEEP_TIME)

    message = self.vehicle.message_factory.set_attitude_target_encode(
      0,                                 # Timestamp in milliseconds since system boot (not used).
      0,                                 # System ID
      0,                                 # Component ID
      self.STANDARD_ATTITUDE_BIT_FLAGS,  # Bit flags.
      attitude.quaternion,               # Attitude quaternion.
      0,                                 # Body roll rate.
      0,                                 # Body pitch rate.
      0,                                 # Body yaw rate.
      thrust                             # Collective thrust, from 0-1.
    )
    self.vehicle.send_mavlink(message)
    self.vehicle.commands.upload()
    self.last_attitude = attitude
    self.last_thrust = thrust

  def hover(self):
    pass

  def takeoff_attitude(self, target_altitude):
    """
    @purpose: Takeoff using thrusts specific with SET_ATTITUDE_TARGET
    @args:
      attitude: Target altitude in meters.
    @returns:
    """
    self.STATE = VehicleStates.takeoff

    self.initial_yaw = self.vehicle.attitude.yaw

    self.arm_drone()
    self.switch_control()

    initial_alt = self.vehicle.location.global_relative_frame.alt

    while((self.vehicle.location.global_relative_frame.alt - initial_alt) < target_altitude):
      self.set_angle_thrust(DroneAttitude(0,0, math.radians(self.initial_yaw)), StandardThrusts.takeoff)
      sleep(self.STANDARD_SLEEP_TIME)

    print('Reached target altitude:{0:.2f}m'.format(self.vehicle.location.global_relative_frame.alt))


  def land(self):
    """
    @purpose: Initiate a landing using the built-in ArduPilot mode.
    @args:
    @returns:
    """
    self.vehicle.mode = dronekit.VehicleMode("LAND")
    self.STATE = VehicleStates.landing
    while((self.vehicle.location.global_relative_frame.alt) >= self.LAND_ALTITUDE):
      sleep(self.STANDARD_SLEEP_TIME)
    else:
      self.STATE = VehicleStates.landed

  def check_battery_voltage(self):
    """
    @purpose: Check battery voltage.
    @args:
    @returns:
    """
    if(self.vehicle.battery.voltage < self.BATTERY_FAILSAFE_VOLTAGE):
        self.land()

class FailsafeController(threading.Thread):

  def __init__(self, atc_instance):
    self.atc = atc_instance
    self.stoprequest = threading.Event()
    super(FailsafeController, self).__init__()

  def run(self):
    while not self.stoprequest.isSet():
      if self.atc.STATE == VehicleStates.hover or self.atc.STATE == VehicleStates.flying:
        self.atc.check_battery_voltage()
      sleep(STANDARD_SLEEP_TIME) 

  def join(self, timeout=None):
    if self.atc.vehicle.armed:
      if self.atc.STATE != VehicleStates.landed:
        self.atc.land()
    self.stoprequest.set()
    super(FailsafeController, self).join(timeout)
