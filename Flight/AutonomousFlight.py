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
  PITCH_P = 10.00
  PITCH_I = 0.00
  PITCH_D = 15.00
  ROLL_P = 10.00
  ROLL_I = 0.00
  ROLL_D = 15.00
  YAW_P = 1.15
  YAW_I = 0.00
  YAW_D = 9.00
  THROTTLE_P = 15.00
  THROTTLE_I = 0.00
  THROTTLE_D = 10.00
  ALTITUDE_P = 0.35
  ALTITUDE_I = 0.11
  ALTITUDE_D = 0.05
  ROLL_CHANNEL = '1'
  PITCH_CHANNEL = '2'
  THROTTLE_CHANNEL = '3'
  YAW_CHANNEL = '4'
  PID_SAMPLE_TIME = 0.00
  ALT_PID_SAMPLE_TIME = 0.01

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
      self.Yaw_PID.setSampleTime(self.PID_SAMPLE_TIME)

      self.Throttle_PID = PID.PID(self.THROTTLE_P, self.THROTTLE_I, self.THROTTLE_D)
      self.Throttle_PID.SetPoint = 0.00
      self.Throttle_PID.setSampleTime(self.PID_SAMPLE_TIME)

      self.Altitude_PID = PID.PID(self.ALTITUDE_P, self.ALTITUDE_I, self.ALTITUDE_D)
      self.Altitude_PID.SetPoint = 0.00
      self.Altitude_PID.setSampleTime(self.ALT_PID_SAMPLE_TIME)

      self.controllers_initialized = True

  def send_velocity_vector(self, requested_flight_vector, desired_altitude = None, desired_yaw = None):
    self.Pitch_PID.SetPoint = requested_flight_vector.x
    self.Roll_PID.SetPoint = requested_flight_vector.y
    self.Throttle_PID.SetPoint = requested_flight_vector.z

    if(desired_yaw and requested_flight_vector.magnitude() == 0.00):
      # By checking the magnitude, we ensure that the vehicle will only yaw while it has a velocity of 0, 0, 0, for all axes.
      # There is a similar check below in update_controllers.
      self.Yaw_PID.SetPoint = self.get_yaw_radians(desired_yaw)

    if(desired_altitude):
      self.Altitude_PID.SetPoint = desired_altitude

  def update_controllers(self):

    #Start by updating our z-axis controllers regardless of state and updating the PWM value.
    self.Altitude_PID.update(self.atc.get_altitude())
    self.Altitude_PWM = self.convert_altitude_to__PWM(self.Altitude_PID.output)

    # self.Throttle_PID.update(self.atc.vehicle.velocity[2])
    # self.Throttle_PWM += self.constrain_rc_values(self.Throttle_PID.output)

    #Next, set PWM values for other axes to neutral. The other axes controllers and PWM values will only be activated and changed if the current vehicle state requires it.
    self.Roll_PWM = self.ROLL_MID
    self.Pitch_PWM = self.PITCH_MID
    self.Yaw_PWM = self.YAW_MID

    if(self.atc.STATE != "TAKEOFF" and self.atc.STATE != "LANDING" and self.atc.STATE != "LANDED"):
      #Vehicle states such as landing or takeoff can adversly affect the any pitch, roll, or yaw controller so they will not be activated in these states.
      #TODO Figure out why VehicleStates can't be imported here.
      if("HOVER" in self.atc.STATE):
        #Yaw (besides the altitude/throttle controller itself) is the only other controller that is allowed to be updated while in hover.
        self.Yaw_PID.update(self.atc.vehicle.attitude.yaw)                
        self.Yaw_PWM += self.Yaw_PID.output
      else:
        #The vehicle will drift if the other (Pitch and Roll) controllers are on in hover. This may be caused by the tuning of the respective PID controllers.
        #In any case, the tunings don't particularly matter. LOITER mode will stop the vehicle if we return the Pitch and Roll channel to neutral.
        if(self.atc.STATE == "FLYING"):
          self.Pitch_PID.update(self.atc.vehicle.velocity[0])
          self.Roll_PID.update(self.atc.vehicle.velocity[1])
          self.Pitch_PWM -= self.Pitch_PID.output
          self.Roll_PWM += self.Roll_PID.output
        elif(self.atc.STATE == "FLYING_PITCH"):
          self.Pitch_PID.update(self.atc.vehicle.velocity[0])
          self.Pitch_PWM -= self.Pitch_PID.output
          #Roll will take the ROLL_MID value as set above. This ensures that the vehicle only travel along the requested axis.
        elif(self.atc.STATE == "FLYING_ROLL"):
          self.Roll_PID.update(self.atc.vehicle.velocity[1])
          self.Roll_PWM += self.Roll_PID.output
          #Pitch will take the PITCH_MID value as set above. This ensures that the vehicle only travel along requested axis.
    
    self.Pitch_PWM = self.constrain_rc_values(self.Pitch_PWM)
    self.Roll_PWM = self.constrain_rc_values(self.Roll_PWM)
    self.Yaw_PWM = self.constrain_rc_values(self.Yaw_PWM)

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

    # self.atc.vehicle.channels.overrides[self.THROTTLE_CHANNEL] = self.Throttle_PWM
    self.atc.vehicle.channels.overrides[self.THROTTLE_CHANNEL] = self.Altitude_PWM

  def get_yaw_radians(self, angle):
    if angle < 180:
        return math.radians(angle)
    else:
        return math.radians(angle-180) - math.pi

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

  # def get_debug_string(self):
  #   os.system("clear")
  #   print("Vehicle State: " + self.atc.STATE + 
  #   # "\n\nZ Velocity Controller Out: " + str(self.atc.pid_flight_controller.Throttle_PID.output) + 
  #   # "\nZ Velocity RC Out: " + str(self.atc.pid_flight_controller.Throttle_PWM) + 
  #   # "\nVehicle Z Velocity: " + str(self.atc.vehicle.velocity[2]) + 
  #   # "\nTarget Z Velocity: " + str(self.atc.pid_flight_controller.Throttle_PID.SetPoint) + 
  #   "\n\nAltitude Controller Out: " + str(self.atc.pid_flight_controller.Altitude_PID.output) + 
  #   "\nAltitude RC Out: " + str(self.atc.pid_flight_controller.Altitude_PWM) + 
  #   "\nVehicle Altitude: " + str(self.atc.get_altitude()) + 
  #   "\n\nPitch Controller Out: " + str(self.atc.pid_flight_controller.Pitch_PID.output) + 
  #   "\nPitch RC Out: " + str(self.atc.pid_flight_controller.Pitch_PWM) + 
  #   "\nVehicle X Velocity: " + str(self.atc.vehicle.velocity[0]) + 
  #   "\nTarget X Velocity: " + str(self.atc.pid_flight_controller.Pitch_PID.SetPoint) + 
  #   "\n\nRoll Controller Out: " + str(self.atc.pid_flight_controller.Roll_PID.output) + 
  #   "\nRoll RC Out: " + str(self.atc.pid_flight_controller.Roll_PWM) + 
  #   "\nVehicle Y Velocity: " + str(self.atc.vehicle.velocity[1]) + 
  #   "\nTarget Y Velocity: " + str(self.atc.pid_flight_controller.Roll_PID.SetPoint) + 
  #   "\n\nYaw Controller Out: " + str(self.atc.pid_flight_controller.Yaw_PID.output) + 
  #   "\nYaw RC Out: " + str(self.atc.pid_flight_controller.Yaw_PWM) + 
  #   "\nVehicle Yaw: " + str(math.degrees(self.atc.vehicle.attitude.yaw)) + 
  #   "\nTarget Yaw: " + str(math.degrees(self.atc.pid_flight_controller.Yaw_PID.SetPoint)))
  # return debug_string
