import time
import math
import numpy as np
from PID import PID

class PIDValue():
  def __init__(self, p, i, d):
    self.p = p
    self.i = i
    self.d = d

  @property
  def p(self):
    return self._p

  @p.setter
  def p(self, value):
    self._p = value
  
  @property
  def i(self):
    return self._i
  
  @i.setter
  def i(self, value):
    self._i = value
  
  @property
  def d(self):
    return self._d

  @d.setter
  def d(self, value):
    self._d = value

class PIDFlightController(object):
  YAW_MID = 1494.0
  PITCH_MID = 1494.0
  ROLL_MID = 1494.0
  THROTTLE_MIN = 982.0
  THROTTLE_MAX = 2006.0
  PITCH_PID = PIDValue(1.8125, 0., 4.625)
  ROLL_PID = PIDValue(1.8125, 0., 4.625)
  YAW_PID = PIDValue(.73, 0., 8.)
  THROTTLE_PID = PIDValue(1.8125, 0., 4.625)
  ALTITUDE_PID = PIDValue(.39, .09, .05)
  ROLL_CHANNEL = '1'
  PITCH_CHANNEL = '2'
  THROTTLE_CHANNEL = '3'
  YAW_CHANNEL = '4'
  PID_SAMPLE_TIME = 0.01
  CONTROLLERS = [('pitch', PITCH_PID), ('roll', ROLL_PID), ('yaw', YAW_PID), ('throttle', THROTTLE_PID), ('altitude', ALTITUDE_PID)]

  def __init__(self, atc):
    self.atc = atc

    self.throttle_pwm = self.THROTTLE_MIN
    self.altitude_pwm = self.THROTTLE_MIN
    self.roll_pwm = self.ROLL_MID
    self.pitch_pwm = self.PITCH_MID
    self.yaw_pid = self.YAW_MID
    
    for controller_name, pid in PIDFlightController.CONTROLLERS:
      attribute_name = '%s_pid' % (controller_name,)
      setattr(self, attribute_name, PID(pid.p, pid.i, pid.d))
      
      controller = getattr(self, attribute_name)
      controller.SetPoint = 0.
      controller.setSampleTime(self.PID_SAMPLE_TIME)

  @property
  def target_velocity(self):
    return np.array([self.pitch_pid.SetPoint, self.roll_pid.SetPoint, self.throttle_pid.SetPoint])

  @target_velocity.setter
  def target_velocity(self, velocity):
    self.pitch_pid.SetPoint, self.roll_pid.SetPoint, self.throttle_pid.SetPoint = velocity

  @property
  def target_altitude(self):
    return self.altitude_pid.SetPoint

  def clear_pitch_controller(self):
    self.pitch_pid.output = 0.
    self.pitch_pwm = self.PITCH_MID
  
  def clear_roll_controller(self):
    self.roll_pid.output = 0.
    self.roll_pwm = self.ROLL_MID

  @target_altitude.setter
  def target_altitude(self, value):
    assert value > 0, 'Altitude must be positive'
    self.altitude_pid.SetPoint = value

  def update_controllers(self):
    x_vel, y_vel, z_vel = self.atc.vehicle.velocity

    self.altitude_pid.update(self.atc.altitude)
    self.altitude_pwm = self.convert_altitude_to_pwm(self.altitude_pid.output)
    
    if self.target_velocity[2] and 'TAKEOFF' not in self.atc.state:
      self.altitude_pid.SetPoint = self.atc.altitude
    
    self.throttle_pid.update(z_vel)
    self.throttle_pwm += self.throttle_pid.output

    if math.pi/2 <= np.abs(self.atc.vehicle.attitude.yaw % (2*math.pi)) <= 3*math.pi/2:
      x_vel, y_vel = (-x_vel, -y_vel)

    if "HOVER" in self.atc.state:
      self.clear_pitch_controller()
      self.clear_roll_controller()
    elif "FLYING" in self.atc.state:
      if not self.pitch_pid.SetPoint:
        self.clear_pitch_controller()
      else:
        self.pitch_pid.update(x_vel)
        self.pitch_pwm -= self.pitch_pid.output
      
      if not self.roll_pid.SetPoint:
        self.clear_roll_controller()
      else:
        self.roll_pid.update(y_vel)
        self.roll_pwm += self.roll_pid.output

  def write_to_rc_channels(self, should_flush_channels=False):
    if should_flush_channels:
      self.roll_pwm = self.ROLL_MID
      self.pitch_pwm = self.PITCH_MID
      self.yaw_pwm = self.YAW_MID
      self.throttle_pwm = self.THROTTLE_MIN
      self.altitude_pwm = self.THROTTLE_MIN

    fourth_channel_name = 'altitude' if 'TAKEOFF' not in self.atc.state else 'throttle'
    channels = zip(('pitch', 'roll', 'yaw', fourth_channel_name), (self.PITCH_CHANNEL, self.ROLL_CHANNEL, self.YAW_CHANNEL, self.THROTTLE_CHANNEL))
    for name, channel in channels:
        self.atc.vehicle.channels.overrides[channel] = self.constrain_rc_values(getattr(self, '%s_pwm' % (name,)))

  def convert_altitude_to_pwm(self, desired_altitude):
    rc_out =  340.0 * desired_altitude + 986.0
    return min(max(self.THROTTLE_MIN, rc_out), self.THROTTLE_MAX)

  def constrain_rc_values(self, rc_out):
    return min(max(self.THROTTLE_MIN, rc_out), self.THROTTLE_MAX)