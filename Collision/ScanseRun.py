from time import sleep
from altScan import LIDAR
import dronekit

vehicle = dronekit.connect("/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00", wait_ready=True)

'''def distance_sensor(min_dist, max_dist, current_dist, sector):
    msg = vehicle.message_factory.distance_sensor_encode(0, 0 , 0, 
                            0, min_dist, max_dist, current_dist, 0, 0, sector,0)
    print "%s" %msg
    vehicle.send_mavlink(msg)
    vehicle.commands.upload()'''

def send_lidar_message(min_dist, max_dist, current_dist, sector):
    print("Distance :" + str(current_dist) + " Quad: " + str(sector))
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

blah = LIDAR()
blah.connect_to_lidar()

blah.reset_sectors()
while(1):   #constantly grab data
    retVal = blah.get_lidar_data()
    secval = 0
    for sector in retVal:
        print "\nFor sector " + (str)(secval)
        endVal = min(10, len(sector))
        for val in range(0,endVal):
            print "Sending message"
            send_lidar_message(10, 300, sector[val][0], sector[val][2]) #((8 - sector[val][2] % 8))
        secval += 1
        sleep(0.000001)

    