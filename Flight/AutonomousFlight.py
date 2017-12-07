import PID, time, math

class FlightVector(object):
  def __init__(self, x, y, z):
    self.x = float(x)
    self.y = float(y)
    self.z = float(z)

  # String representation
  def __str__(self):
    return '<%s, %s, %s>' % (self.x, self.y, self.z)

  # Produce a copy of itself
  def __copy(self):
    return FlightVector(self.x, self.y, self.z)

  # Signing
  def __neg__(self):
    return FlightVector(-self.x, -self.y, -self.z)

  # Scalar multiplication
  def __mul__(self, number):
    return FlightVector(self.x * number, self.y * number, self.z * number)

  def __rmul__(self, number):
    return self.__mul__(number)

  # Division
  def __div__(self, number):
    return self.__copy() * (number**-1)

  # Arithmetic operations
  def __add__(self, operand):
    return FlightVector(self.x + operand.x, self.y + operand.y, self.z + operand.z)

  def __sub__(self, operand):
    return self.__copy() + -operand

  # Cross product
  # cross = a ** b
  def __pow__(self, operand):
    return FlightVector(self.y*operand.z - self.z*operand.y, 
                      self.z*operand.x - self.x*operand.z, 
                      self.z*operand.y - self.y*operand.x)

  # Dot product
  # dp = a & b
  def __and__(self, operand):
    return (self.x * operand.x) + \
            (self.y * operand.y) + \
            (self.z * operand.z)

  # Operations
  def normal(self):
    return self.__copy() / self.magnitude()

  def magnitude(self):
    return (self.x**2 + self.y**2 + self.z**2)**(.5)

class PIDFlightController(object):
  YAW_MID = 1494.0
  PITCH_MID = 1494.0
  ROLL_MID = 1494.0
  THROTTLE_MIN = 982.0
  THROTTLE_MAX = 2006.0
  PITCH_P = 1.39
  PITCH_I = 0.0
  PITCH_D = 3.70
  ROLL_P = 1.30
  ROLL_I = 0.00
  ROLL_D = 3.70
  YAW_P = 0.73
  YAW_I = 0.00
  YAW_D = 8.00
  THROTTLE_P = 2.95
  THROTTLE_I = 0.00
  THROTTLE_D = 2.75
  ALTITUDE_P = 0.39
  ALTITUDE_I = 0.09
  ALTITUDE_D = 0.05
  ROLL_CHANNEL = '1'
  PITCH_CHANNEL = '2'
  THROTTLE_CHANNEL = '3'
  YAW_CHANNEL = '4'
  PID_SAMPLE_TIME = 0.01
  YAW_PID_SAMPLE_TIME = 0.00

  def __init__(self, atc):
    self.atc = atc
    self.controllers_initialized = False

    self.Throttle_PID = None
    self.Altitude_PID = None
    self.Roll_PID = None
    self.Pitch_PID = None
    self.Yaw_PID = None
    self.Throttle_PWM = self.THROTTLE_MIN
    self.Altitude_PWM = self.THROTTLE_MIN
    self.Roll_PWM = self.ROLL_MID
    self.Pitch_PWM = self.PITCH_MID
    self.Yaw_PWM = self.YAW_MID
    self.initialize_controllers()

  def initialize_controllers(self):
    if not self.controllers_initialized:
      self.Pitch_PID = PID.PID(self.PITCH_P, self.PITCH_I, self.PITCH_D)
      self.Pitch_PID.SetPoint = 0.00
      self.Pitch_PID.setSampleTime(self.PID_SAMPLE_TIME)

      self.Roll_PID = PID.PID(self.ROLL_P, self.ROLL_I, self.ROLL_D)
      self.Roll_PID.SetPoint = 0.00
      self.Roll_PID.setSampleTime(self.PID_SAMPLE_TIME)
      
      self.Yaw_PID = PID.PID(self.YAW_P , self.YAW_I, self.YAW_D)
      self.Yaw_PID.SetPoint = 0.00
      self.Yaw_PID.setSampleTime(self.YAW_PID_SAMPLE_TIME)

      self.Throttle_PID = PID.PID(self.THROTTLE_P, self.THROTTLE_I, self.THROTTLE_D)
      self.Throttle_PID.SetPoint = 0.00
      self.Throttle_PID.setSampleTime(self.YAW_PID_SAMPLE_TIME)

      self.Altitude_PID = PID.PID(self.ALTITUDE_P, self.ALTITUDE_I, self.ALTITUDE_D)
      self.Altitude_PID.SetPoint = 0.00
      self.Altitude_PID.setSampleTime(self.PID_SAMPLE_TIME)

      self.controllers_initialized = True

  def send_velocity_vector(self, requested_flight_vector, desired_altitude = None, desired_yaw = None):

    self.Pitch_PID.SetPoint = requested_flight_vector.x
    self.Roll_PID.SetPoint = requested_flight_vector.y
    self.Throttle_PID.SetPoint = requested_flight_vector.z

    if(desired_yaw is not None and 
      requested_flight_vector.x == 0.00 and 
      requested_flight_vector.y ==0):
      # By checking the magnitude, we ensure that the vehicle will only yaw while it is not moving horizontally.
      # There is a similar check below in update_controllers.
      self.Yaw_PID.SetPoint = self.get_yaw_radians(desired_yaw)

    if(desired_altitude):
      #Only use the desired altitude arguemnt if the Altitude_PID controller is enabled.
      self.Altitude_PID.SetPoint = desired_altitude

  def update_controllers(self):
    vehicle_x_velocity = (self.atc.vehicle.velocity[0])
    vehicle_y_velocity = (self.atc.vehicle.velocity[1])
    vehicle_z_velocity = (self.atc.vehicle.velocity[2])

    #Start by updating our z-axis controllers regardless of state and updating the PWM value.
    # self.Altitude_PID.update(self.atc.get_altitude())
    # self.Altitude_PWM = self.convert_altitude_to__PWM(self.Altitude_PID.output)

    self.Throttle_PID.update(vehicle_z_velocity)
    self.Throttle_PWM += self.Throttle_PID.output

    # #This flips the velocity reeadings so that they are relative to the vehicle and not the world.
    if(math.cos(self.atc.vehicle.attitude.yaw) <= 0.0):
      vehicle_x_velocity *=-1.0
      vehicle_y_velocity *=-1.0

    #TODO Figure out why VehicleStates can't be imported here.

    if("HOVER" in self.atc.STATE):
      #The vehicle will drift if the other (Pitch and Roll) controllers are on in hover.
      #This is likely caused by too much noise in accelerometer data.
      self.Pitch_PID.output = 0.0
      self.Roll_PID.output = 0.0
      self.Pitch_PWM = self.PITCH_MID
      self.Roll_PWM = self.ROLL_MID
      self.Yaw_PID.update(self.atc.vehicle.attitude.yaw)                
      self.Yaw_PWM += self.Yaw_PID.output
      if("(Yaw Achieved)" in self.atc.STATE):
        self.Yaw_PID.output = 0.0
        self.Yaw_PWM = self.YAW_MID
    elif("FLYING" in self.atc.STATE):
      if(self.Pitch_PID.SetPoint == 0.00):
        self.Pitch_PID.output = 0.0
        self.Pitch_PWM = self.PITCH_MID
      else:
        self.Pitch_PID.update(vehicle_x_velocity)
        self.Pitch_PWM -= self.Pitch_PID.output
      if(self.Roll_PID.SetPoint == 0.00):
        self.Roll_PID.output = 0.0
        self.Roll_PWM = self.ROLL_MID
      else:
        self.Roll_PID.update(vehicle_y_velocity)
        self.Roll_PWM += self.Roll_PID.output

    self.Pitch_PWM = self.constrain_rc_values(self.Pitch_PWM)
    self.Roll_PWM = self.constrain_rc_values(self.Roll_PWM)
    self.Yaw_PWM = self.constrain_rc_values(self.Yaw_PWM)
    self.Throttle_PWM = self.constrain_rc_values(self.Throttle_PWM)

  def write_to_rc_channels(self, should_flush_channels=False):
    
    if(should_flush_channels):
      self.Roll_PWM = self.ROLL_MID
      self.Pitch_PWM = self.PITCH_MID
      self.Yaw_PWM = self.YAW_MID
      self.Throttle_PWM = self.THROTTLE_MIN
      self.Altitude_PWM = self.THROTTLE_MIN

    self.atc.vehicle.channels.overrides[self.PITCH_CHANNEL] = self.Pitch_PWM
    self.atc.vehicle.channels.overrides[self.ROLL_CHANNEL] = self.Roll_PWM
    self.atc.vehicle.channels.overrides[self.YAW_CHANNEL] = self.Yaw_PWM

    self.atc.vehicle.channels.overrides[self.THROTTLE_CHANNEL] = self.Throttle_PWM
    # self.atc.vehicle.channels.overrides[self.THROTTLE_CHANNEL] = self.Altitude_PWM

  def get_yaw_radians(self, angle):
    if angle < 180.0:
        if angle == 0.0:
            angle = 0.01
        return math.radians(angle)
    else:
        if angle == 180.0:
            angle = 179.9
            return math.radians(angle)
        return math.radians(angle-180.0) - math.pi

  def convert_altitude_to__PWM(self, desired_altitude):
    rc_out =  340.0 * desired_altitude + 986.0
    if(rc_out < self.THROTTLE_MIN):
      rc_out = self.THROTTLE_MIN
    elif(rc_out > self.THROTTLE_MAX):
      rc_out = self.THROTTLE_MAX
    return rc_out

  def constrain_rc_values(self, rc_out):
    if(rc_out < self.THROTTLE_MIN):
      rc_out = self.THROTTLE_MIN
    elif(rc_out > self.THROTTLE_MAX):
      rc_out = self.THROTTLE_MAX
    return rc_out

  def get_debug_string(self):
    vehicle_x_velocity = (self.atc.vehicle.velocity[0])
    vehicle_y_velocity = (self.atc.vehicle.velocity[1])

    debug_string = ("Vehicle State: " + self.atc.STATE + 
    "\n\nZ Velocity Controller Out: " + str(self.Throttle_PID.output) + 
    "\nZ Velocity RC Out: " + str(self.Throttle_PWM) + 
    "\nVehicle Z Velocity: " + str(self.atc.vehicle.velocity[2]) + 
    "\nTarget Z Velocity: " + str(self.Throttle_PID.SetPoint) + 
    # "\n\nAltitude Controller Out: " + str(self.Altitude_PID.output) + 
    # "\nAltitude RC Out: " + str(self.Altitude_PWM) + 
    "\nVehicle Altitude: " + str(self.atc.get_altitude()) + 
    # "\nWithin Alt Threshold: " + str(self.atc.in_range(self.atc.ALT_PID_THRESHOLD, self.Altitude_PID.SetPoint, self.atc.get_altitude())) +
    "\n\nPitch Controller Out: " + str(self.Pitch_PID.output) + 
    "\nPitch RC Out: " + str(self.Pitch_PWM) + 
    "\nVehicle X Velocity: " + str(vehicle_x_velocity) + 
    "\nTarget X Velocity: " + str(self.Pitch_PID.SetPoint) + 
    "\n\nRoll Controller Out: " + str(self.Roll_PID.output) + 
    "\nRoll RC Out: " + str(self.Roll_PWM) + 
    "\nVehicle Y Velocity: " + str(vehicle_y_velocity) + 
    "\nTarget Y Velocity: " + str(self.Roll_PID.SetPoint) + 
    "\n\nYaw Controller Out: " + str(self.Yaw_PID.output) + 
    "\nYaw RC Out: " + str(self.Yaw_PWM) + 
    "\nVehicle Yaw: " + str(self.atc.get_yaw_deg()) + 
    "\nTarget Yaw: " + str(math.degrees(self.Yaw_PID.SetPoint)) +
    "\nWithin Yaw Threshold: " + str(self.atc.in_range(self.atc.YAW_PID_THRESHOLD, math.degrees(self.Yaw_PID.SetPoint), self.atc.get_yaw_deg())))
    return debug_string
