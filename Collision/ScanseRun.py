from time import sleep
from altscan import LIDAR
import dronekit
import StateSync as Sync

HIGH_ALT = 2.5
MED_ALT = 2.0
LOW_ALT = 1.5

stateSync = Sync.StateSync("collision")

#vehicle = dronekit.connect("/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00", wait_ready=True)

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
                coll_msg = {
                    'first':10,
                    'second':300,
                    'third':sector[val][0],
                    'four':sector[val][2]
                }
                stateSync.sendCollMsg(height, coll_msg)
                #send_lidar_message(10, 300, sector[val][0], sector[val][2]) #((8 - sector[val][2] % 8)) -- This is now implemented in altScan.py
        secval += 1
        sleep(0.000001)

    
