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
    print "SCANSE INIT"
    self.sweep = None

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

    # get_scans is coroutine-based generator lazily returning scans
    for scan in itertools.islice(self.sweep.get_scans(), 1): #for every sector
      
      sector_points = []
      sector_lists = [ [], [], [], [], [], [], [], [] ]
      
      #sort data

      for sample in scan.samples: #for every sample
        
        distance = sample.distance
        angle_deg = (sample.angle / 1000.0) % 360.0
        angle_rad = math.radians(sample.angle / 1000.0)
        sector = ((angle_deg % 360.0) // self.QUADRANT_SIZE)
        
        if (distance < self.MAX_SAFE_DISTANCE and distance > self.MIN_SAFE_DISTANCE):
          sector_lists[(int)(sector)].append( (distance, angle_deg) )
          #print (distance, angle_deg)
          
    
    for k in range(0, 8): #force at least 5 elements into list, they will be filtered out
        sector_lists[k] = [len(sector_lists[k]), (3000, -1.0)]*5
        
    #Begin processing
    for i in range(0, 8):
      sector_lists[i].sort()
      #print sector_lists[i]
      first_five = []
      for j in range(0, 5):
        first_five.append(sector_lists[i][j])
      print first_five
      lidar_data[i]=first_five

    print("\n")

    return lidar_data

  def shutdown(self):
    self.sweep.stop_scanning()
    self.sweep.__exit__()

