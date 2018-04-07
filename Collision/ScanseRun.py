from time import sleep
from altscan import LIDAR
import dronekit
import RPi.GPIO as GPIO
import StateSync as Sync

HIGH_ALT = 2.5
MED_ALT = 2.0
LOW_ALT = 1.5

stateSync = Sync.StateSync("collision")

#vehicle = dronekit.connect("/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00", wait_ready=True)
class Sonar:

    #default constructor
    #trigger, echo, and side are passes through and set accordingly
    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo
        #set GPIO direction (IN / OUT)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    #performs operations to get the approximate distance
    #returns distance to calling function
    def get_distance(self):
        GPIO.output(self.trigger, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        self.startTime = time.time()
        self.stopTime = time.time()

        # save StartTime
        while GPIO.input(self.echo) == 0:
            self.startTime = time.time()

        # save time of arrival
        while GPIO.input(self.echo) == 1:
            self.stopTime = time.time()

        # time difference between start and arrival
        self.timePassed = self.stopTime - self.startTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distance = (self.timePassed * 34300) / 2

        return self.distance
    
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

MIN_SONAR_DISTANCE = 3
MAX_SONAR_DISTANCE = 4000
TRIG_PIN = 2
ECHO_PIN = 3

def alt_determinate(distance, sector):
    if ((distance > 1.0 and distance < 1.5) and (sector == 2 or sector == 3 or sector == 4 or sector == 5):
      return LOW_ALT
    elif((distance > 0.5 and distance < 1.0) and (sector == 1 or sector == 6)):
      return HIGH_ALT
    elif((sector == 1 or sector == 6) and (distance > 1.0)):
      return MED_ALT
    elif(((distance > 0.5) and distance <= 1.0) and (sector not 0 and sector not 7)):
      return MED_ALT
    else:
      return HIGH_ALT

def send_lidar_message(min_dist, max_dist, current_dist, sector):
    print("Distance :" + str(current_dist) + " Quad: " + str(sector) + "Speed" + str(vel)
    message = vehicle.message_factory.distance_sensor_encode(
    0,                                             # time since system boot, not used
    min_dist,                                      # min distance cm
    max_dist,                                      # max distance cm
    current_dist,                                  # current distance, must be int
    0,                                             # type = laser
    0,                                             # onboard id, not used
    sector,                                        # sensor rotation
    0                                              # covariance, not used
    )
    vehicle.send_mavlink(message)
    vehicle.commands.upload()



def send_distance_message(distance_to_ground):
    message = self.vehicle.message_factory.distance_sensor_encode(
        0,                                             # time since system boot, not used
        MIN_SONAR_DISTANCE,                            # min distance cm
        MAX_SONAR_DISTANCE,                            # max distance cm
        distance_to_ground,                            # current distance, must be int
        0,                                             # type = laser
        0,                                             # onboard id, not used
        MAV_SENSOR_ROTATION_PITCH_270,                 # Downward facing range sensor.
        0                                              # covariance, not used
    )
    self.vehicle.send_mavlink(message)
    self.vehicle.commands.upload() 

    
sleep(0.1)

downward_sonar = Sonar(TRIG_PIN, ECHO_PIN)
lidar = LIDAR()
lidar.connect_to_lidar()

lidar.reset_sectors()
while(1):   #constantly grab data
    retVal = lidar.get_lidar_data()
    secval = 0
    for sector in retVal:   
        print "\nFor sector " + (str)(secval)
        endVal = min(10, len(sector))
        if (endVal >= 5):
            height = 0
            for val in range(0,endVal):
                det_height = alt_determinate(sector[val][0], sector[val][2])
                height = max(height, det_height)
                print "Sending message"
                stateSync.sendCollision(height)
                #send_lidar_message(10, 300, sector[val][0], sector[val][2]) #((8 - sector[val][2] % 8)) -- This is now implemented in altScan.py
        secval += 1
        sleep(0.000001)
    send_distance_message(downward_sonar.get_distance())

    
