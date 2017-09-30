from time import sleep
from Scanse import LIDAR

blah = LIDAR()
blah.connect_to_lidar()

while(1):   #constantly grab data
    blah.get_lidar_data()