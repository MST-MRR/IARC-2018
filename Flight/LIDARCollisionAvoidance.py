from time import sleep
import itertools
import sys
import math

from sweeppy import Sweep

class LIDAR():
  MIN_SAFE_DISTANCE = 10.0
  MAX_SAFE_DISTANCE = 300.0
  QUADRANT_SIZE = 45.0

  def __init__(self):
    self.lidar_sensor = "/dev/cu.usbserial-DO00867Q" #this is for Mac OS X 
    #self.lidar_sensor = "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO00867Q-if00-port0" #this is for Linux probably
    self.sweep = Sweep(self.lidar_sensor)
    #print "SCANSE INIT"
    #self.sweep = None

  def connect_to_lidar(self):
    self.sweep.__enter__()
    # Starts scanning as soon as the motor is ready
    self.sweep.set_motor_speed(6)
    self.sweep.set_sample_rate(1000)
    self.sweep.start_scanning()

  #We encountered errors with getting points in each sector, so this method
  #     forces the sensor to be aligned with sector 0 before providing useful data
  #     Run this method before getting the first sample
  def reset_sectors(self):
    for scan in itertools.islice(self.sweep.get_scans(), 1): #for every sector
      print ""

  def get_lidar_data(self):
    # Starts scanning as soon as the motor is ready
    lidar_data = [ [], [], [], [], [], [], [], [] ]
    #print "check"
    # get_scans is coroutine-based generator lazily returning scans
    for scan in itertools.islice(self.sweep.get_scans(), 1): #for every sector
      all_points = []
      sector_points = []
      sector_lists = [ [], [], [], [], [], [], [], [] ]
      
      #sort data

      for sample in scan.samples: #for every sample
        
        distance = sample.distance
        angle_deg = (sample.angle / 1000.0) % 360.0
        sector = (int)((angle_deg % 360.0) // self.QUADRANT_SIZE)
        
        if (distance < self.MAX_SAFE_DISTANCE and distance > self.MIN_SAFE_DISTANCE):
          sector_lists[(int)(sector)].append( (distance, angle_deg, sector) )

    for point_list in sector_lists:
      point_list.sort()

    for sector_index in range(0,8):
      endVal = min(5, len(sector_lists[sector_index]))
      for index in range(0,endVal):
        lidar_data[sector_index].append(sector_lists[sector_index][index])

    return lidar_data

  def distance_sensor(min_dist, max_dist, current_dist, sector):
    msg = message_factory.command_long_encode(0, 0 , mavutil.mavlink.DISTANCE_SENSOR, 
                            0, min_dist, max_dist, current_dist, 0, 0, sector,0)
    print "%s" %msg
    send_mavlink(msg)

  def send_lidar_message(min_dist, max_dist, current_dist, sector):

      distance = data[0]
      sensor_rotation = data[1]
      print("Distance :" + str(distance) + " Quad: " + str(sensor_rotation))
      message = self.vehicle.message_factory.distance_sensor_encode(
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

  def shutdown(self):
    self.sweep.stop_scanning()
    self.sweep.__exit__()
