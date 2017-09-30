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
    #self.lidar_sensor = "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO00867Q-if00-port0" #this is for Linux
    self.sweep = Sweep(self.lidar_sensor)
    self.sweep = None

  def connect_to_lidar(self):
    self.sweep.__enter__()
    # Starts scanning as soon as the motor is ready
    self.sweep.set_motor_speed(6)
    self.sweep.set_sample_rate(1000)

    self.sweep.start_scanning()

  def get_lidar_data(self):
    # Starts scanning as soon as the motor is ready
    lidar_data = []

    # get_scans is coroutine-based generator lazily returning scans ad infinitum
    for scan in itertools.islice(self.sweep.get_scans(), 1):
      sector_dist = [300, 300, 300, 300, 300, 300, 300, 300]
      
      for sample in scan.samples:
        distance = sample.distance
        angle_deg = (sample.angle / 1000.0) % 360.0
        angle_rad = math.radians(sample.angle / 1000.0)
        # x = math.cos(angle_rad) * distance
        # y = math.sin(angle_rad) * distance

        if (distance < self.MAX_SAFE_DISTANCE and distance > self.MIN_SAFE_DISTANCE):
          lidar_data += ([distance, ((angle_deg % 360.0) // self.QUADRANT_SIZE) ])

          if(sector_dist[int(((angle_deg % 360.0) // self.QUADRANT_SIZE))] > distance):
            sector_dist[int(((angle_deg % 360.0) // self.QUADRANT_SIZE))] = distance
          
          
        #Print out values for each sector
        for i, val in enumerate(sector_dist):
          print "[" + str(sector_dist[i]) + ", " + str(i) + "]"
        print("\n")
          
    print("\n")

    return lidar_data

  def shutdown(self):
    self.sweep.stop_scanning()
    self.sweep.__exit__()

