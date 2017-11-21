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

import dronekit
import math
import os
import time
import threading

import AutonomousFlight

class StandardFlightVectors(object):
  hover = AutonomousFlight.FlightVector(0.000, 0.000, 0.000)

class VehicleStates(object):
  hover = "HOVER"
  hover_adjusting = "HOVER (Adjusting vehicle's yaw or altitude)"
  hover_yaw_achieved = "HOVER (Yaw Achieved)"
  flying_pitching = "FLYING (Pitch)"
  flying_rolling = "FLYING (Roll)"
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
  ALT_PID_THRESHOLD = 0.05
  VEL_PID_THRESHOLD = 0.09
  YAW_PID_THRESHOLD = 1.00
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
        stdout = self.flight_log
      
      try:
        print("\nConnecting to vehicle...")        
        self.vehicle = dronekit.connect(self.SIM, wait_ready=True)
      except:
        print("\nUnable to connect to vehicle.")
        return

      self.start_time = int(time.time())
      self.vehicle_initialized = True
      self.failsafes = FailsafeController(self)
      self.failsafes.start()
      self.pid_flight_controller = AutonomousFlight.PIDFlightController(self)
      self.pid_flight_controller.write_to_rc_channels(should_flush_channels=True)
      self.pid_flight_controller.initialize_controllers()

      if(self.get_altitude() < self.LAND_ALTITUDE):
        self.STATE = VehicleStates.landed
      else:
        self.land()

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
      self.pid_flight_controller = AutonomousFlight.PIDFlightController(self)
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
      self.STATE != VehicleStates.avoidance and 
      self.STATE != VehicleStates.unknown)

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

  def get_yaw_deg(self):
    """
    @purpose: Get vehicle yaw in degrees.
    @args:
    @returns: Vehicle yaw in degrees.
    """
    if(self.vehicle):
      return math.degrees(self.vehicle.attitude.yaw)

  def in_range(self, threshold, base_value, num):
    """
    @purpose: Check if number is between base_value +- threshold.
    @args:
      threshold: A decimal you want to add/subtract to the base_value.
      base_value: Base value.
      num: Number to be tested.
    @returns: Boolean if num within range.
    """
    return base_value - threshold <= num <= base_value + threshold

  def takeoff(self, desired_altitude, desired_angle=None):
    """
    @purpose: Initiate a takeoff using the altitude based PID.
    @args: 
      desired_altitude: Altitude to hover at when the takeoff operation is finished.
    @returns:
    """
    self.switch_control()
    self.arm_drone()

    self.STATE = VehicleStates.takeoff
    self.pid_flight_controller.send_velocity_vector(StandardFlightVectors.hover, desired_altitude)

    while(not (self.in_range(self.ALT_PID_THRESHOLD, desired_altitude, self.get_altitude()))):
      sleep(self.STANDARD_SLEEP_TIME)

    if desired_angle is None:
      desired_angle = self.get_yaw_deg()
      
    self.hover(desired_altitude, desired_angle)

  def fly(self, desired_vector):
    """
    @purpose: Fly the vehicle in a direction with a certain speed.
    @args: 
      desired_vector: FlightVector with direction/speed.
    @returns:
    """

    if((desired_vector.x != 0 and desired_vector.y != 0)):
      self.STATE = VehicleStates.flying
    elif(desired_vector.x):
      self.STATE = VehicleStates.flying_pitching
    elif(desired_vector.y):
      self.STATE = VehicleStates.flying_rolling

    #TODO Remove check. This check is here in case the someone asks for a Z axis change only. 
    #This will not change the vehicle's state because the Z axis controller is not finished/enabled yet.
    if "FLYING" in self.STATE:
      self.pid_flight_controller.send_velocity_vector(desired_vector)
    
  def hover(self, desired_altitude=None, desired_angle=None):
    """
    @purpose: Hover/stop the vehicle in the air. Can also be used to Yaw.
              This method will block until the desired_angle/desired_altitude
              is achieved with 2 decimal places of precision (PID_THRESHOLD).
              For example,if you ask to go to 1 meter, this method will wait until the vehicle
              gets to 1.00 +- PID_THRESHOLD meters.
    @args:
      desired_altitude: Altitude for the vehicle to hover at.
      desired_angle: Angle for the vehicle to yaw to.
    @returns:
    """
    
    #If we don't get arguments, set desired to the vehicle's current readings.
    if desired_altitude is None:
      desired_altitude = self.get_altitude()
    if desired_angle is None:
      desired_angle = self.get_yaw_deg()

    #Send the hover vector without an angle first to stop the vehicle.
    hover_vector = deepcopy(StandardFlightVectors.hover)
    self.pid_flight_controller.send_velocity_vector(hover_vector, desired_altitude)

    # #Wait for vehicle to slow before sending next vector with yaw. (For when PID based slowing is disabled.)
    # sleep(1)
  
    #Wait for vehicle to slow down via PID if it was previous flying. 
    #Once we set the vehicle's state to HOVER, we will completely disable/cutoff the controllers and reset the RC channels.
    while("FLYING" in self.STATE and 
        (not self.in_range(self.VEL_PID_THRESHOLD, 0.00, self.vehicle.velocity[0])
        and (not self.in_range(self.VEL_PID_THRESHOLD, 0.00, self.vehicle.velocity[1])))):
      sleep(self.STANDARD_SLEEP_TIME)

    #Re-send the hover vector with angle.
    self.pid_flight_controller.send_velocity_vector(hover_vector, desired_altitude, desired_angle)

    #Check if vehicle's altitude or yaw are not correct and set state appropriately.
    if(not (self.in_range(self.ALT_PID_THRESHOLD, desired_altitude, self.get_altitude()))
      or (desired_angle is not None and not (self.in_range(self.YAW_PID_THRESHOLD, desired_angle, self.get_yaw_deg())))):
      self.STATE = VehicleStates.hover_adjusting
    else:
      self.STATE = VehicleStates.hover

    #Wait for the vehicle to correct.
    while(not (self.in_range(self.ALT_PID_THRESHOLD, desired_altitude, self.get_altitude()))):
      sleep(self.STANDARD_SLEEP_TIME)
    while(desired_angle is not None and not (self.in_range(self.YAW_PID_THRESHOLD, desired_angle, self.get_yaw_deg()))):
      sleep(self.STANDARD_SLEEP_TIME)
    else:
      self.STATE = VehicleStates.hover_yaw_achieved
      sleep(1) #Wait for AutonomousFlight to query the state.

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
      if self.atc.flight_prereqs_clear():
        self.atc.pid_flight_controller.update_controllers()
        if self.atc.vehicle.armed and self.atc.vehicle.mode.name == "LOITER":
          # self.atc.check_battery_voltage()
          self.atc.pid_flight_controller.write_to_rc_channels()
          os.system("clear")
          print(self.atc.pid_flight_controller.get_debug_string())
      sleep(0.1) 
      #DO NOT CHANGE THIS SLEEP TIME, PID LOOPS IN AUTONOMOUSFLIGHT.PY WILL BECOME UNSTABLE.

  def join(self, timeout=None):
    if self.atc.vehicle.armed:
      if self.atc.STATE != VehicleStates.landed or self.atc.vehicle.mode.name != "LAND":
        self.atc.pid_flight_controller.write_to_rc_channels(should_flush_channels=True)
        self.atc.land()

    self.stoprequest.set()
    super(FailsafeController, self).join(timeout)
