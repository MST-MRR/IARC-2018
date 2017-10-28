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

import dronekit
import math
import os
import time
import threading
import LIDARCollisionAvoidance

from AutonomousFlight import FlightVector, PIDFlightController

class StandardFlightVectors(object):
  hover = FlightVector(0.000, 0.000, 0.000)

class VehicleStates(object):
  hover = "HOVER"
  flying = "FLYING"
  takeoff = "TAKEOFF"
  unknown = "UNKNOWN"
  avoidance = "AVOIDANCE"
  landing = "LANDING"
  landed = "LANDED"

class Tower(object):
  SIM = "tcp:127.0.0.1:5762"
  USB = "/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
  UDP = "192.168.12.1:14550"
  MAC = "/dev/cu.usbmodem1"
  MESSAGE_SLEEP_TIME = 0.01
  STANDARD_SLEEP_TIME = 1
  LAND_ALTITUDE = 0.25
  MAX_ANGLE_ALL_AXIS = 15.0
  BATTERY_FAILSAFE_VOLTAGE_PANIC = 9.25
  BATTERY_FAILSAFE_VOLTAGE_SENTINEL = 13.25

  def __init__(self):
    """
    @purpose: Simply for initializing this object, we are not connected to the flight controller at this point.
    @args:
    @returns:
    """
    self.start_time = 0
    self.flight_log = None
    self.vehicle_initialized = False
    self.vehicle = None
    self.failsafes = None
    self.pid_flight_controller = None
    self.STATE = VehicleStates.unknown
    self.LIDAR_enabled = False
    self.LIDAR_instance = None

  def initialize(self, should_write_to_file=False, should_enable_LIDAR=False):
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

      if(should_enable_LIDAR):
        self.LIDAR_instance = LIDAR()

      print("\nConnecting to vehicle...")
      self.vehicle = dronekit.connect(self.SIM, wait_ready=True)

      if not self.vehicle:
        print("\nUnable to connect to vehicle.")
        return

      self.start_time = int(time.time())
      self.vehicle_initialized = True
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
      self.pid_flight_controller = PIDFlightController(self)
      self.pid_flight_controller.initialize_controllers()
      self.STATE = VehicleStates.landed

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

  def switch_control(self, mode_name="LOITER"):
    """
    @purpose: Switch the mode to LOITER and make sure
             that the failsafe thread is running.
    @args: A vehicle mode name, see ArduPilot documentation
          for available modes.
    @returns:
    """
    if not self.pid_flight_controller:
      self.pid_flight_controller = PIDFlightController(self)
      self.pid_flight_controller.initialize_controllers()
    if not self.failsafes:
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
    if self.vehicle.mode.name != mode_name:
      self.vehicle.mode = dronekit.VehicleMode(mode_name)
      while(self.vehicle.mode.name != mode_name):
        sleep(self.STANDARD_SLEEP_TIME)

  def flight_prereqs_clear(self):
    """
    @purpose: Check that objects are initialized and collision avoidance does not have control.
    @args:
    @returns: Boolean if basic checks are complete.
    """
    return (self.pid_flight_controller != None and
      self.pid_flight_controller.controllers_initialized and
      self.STATE != VehicleStates.avoidance)

  def get_uptime(self):
    """
    @purpose: Get up time of this object.
    @args:
    @returns: Vehicle uptime in seconds since the epoch.
    """
    uptime = int(time.time()) - self.start_time
    return uptime

  def get_altitude(self):
    """
    @purpose: Get vehicle altitude.
    @args:
    @returns: Vehicle altitude in meters.
    """
    if(self.vehicle):
      return self.vehicle.location.global_relative_frame.alt

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


  def takeoff(self, desired_altitude):
    """
    @purpose: Initiate a takeoff using the altitude based PID.
    @args: 
      desired_altitude: Altitude to hover at when the takeoff operation is finished.
    @returns:
    """
    self.switch_control()
    self.arm_drone()

    initial_alt = self.get_altitude()

    self.STATE = VehicleStates.takeoff
    self.pid_flight_controller.send_velocity_vector(StandardFlightVectors.hover, desired_altitude)

    while((self.get_altitude() - initial_alt) < desired_altitude):
      sleep(self.STANDARD_SLEEP_TIME)

    self.hover(desired_altitude)

  def fly(self, desired_vector):
    """
    @purpose: Fly the vehicle in a direction with a certain speed.
    @args: 
      desired_vector: FlightVector with direction/speed.
    @returns:
    """
    self.STATE = VehicleStates.flying
    self.pid_flight_controller.send_velocity_vector(desired_vector)
    
  def hover(self, desired_altitude=None, desired_angle=None):
    """
    @purpose: Hover/stop the vehicle in the air. Can also be used to Yaw.
    @args:
      desired_altitude: Altitude for the vehicle to hover at.
      desired_angle: Angle for the vehicle to yaw to.
    @returns:
    """
    intial_alt = self.get_altitude()

    hover_vector = deepcopy(StandardFlightVectors.hover)
    self.pid_flight_controller.send_velocity_vector(hover_vector, desired_altitude, desired_angle)
  
    while(self.get_altitude() < desired_altitude):
      sleep(self.STANDARD_SLEEP_TIME)
    while(self.get_altitude() > desired_altitude):
      sleep(self.STANDARD_SLEEP_TIME)

    self.STATE = VehicleStates.hover

  def land(self):
    """
    @purpose: Initiate a landing using the built-in ArduPilot mode.
    @args:
    @returns:
    """
    self.vehicle.mode = dronekit.VehicleMode("LAND")
    self.STATE = VehicleStates.landing
    self.pid_flight_controller.send_velocity_vector(deepcopy(StandardFlightVectors.hover), desired_altitude=0)
    while((self.get_altitude()) >= self.LAND_ALTITUDE):
      sleep(self.STANDARD_SLEEP_TIME)
    else:
      self.STATE = VehicleStates.landed

  def check_battery_voltage(self):
    """
    @purpose: Check battery voltage.
    @args:
    @returns:
    """
    if(self.vehicle.battery.voltage < self.BATTERY_FAILSAFE_VOLTAGE_PANIC):
        self.land()

class FailsafeController(threading.Thread):

  def __init__(self, atc_instance):
    self.atc = atc_instance
    self.stoprequest = threading.Event()
    super(FailsafeController, self).__init__()

  def run(self):
    while not self.stoprequest.isSet():
      if self.atc.STATE == VehicleStates.hover or self.atc.STATE == VehicleStates.flying:
        # self.atc.check_battery_voltage()
        pass
      if self.atc.flight_prereqs_clear():
        self.atc.pid_flight_controller.update_controllers()
        if self.atc.vehicle.armed and self.atc.vehicle.mode.name == "LOITER":
          self.atc.pid_flight_controller.write_to_rc_channels()
      sleep(0.1) 

  def join(self, timeout=None):
    if self.atc.vehicle.armed:
      if self.atc.STATE != VehicleStates.landed:
        self.atc.land()
        self.atc.pid_flight_controller.write_to_rc_channels(should_flush_channels=True)

    self.stoprequest.set()
    super(FailsafeController, self).join(timeout)
