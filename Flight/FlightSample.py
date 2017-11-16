from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0)
sleep(10)

in_air_yaw = t.get_yaw_deg()

t.hover(desired_angle=in_air_yaw - 45.0)
sleep(5)

t.hover(desired_angle=in_air_yaw + 45.0)
sleep(5)

t.hover(desired_angle=in_air_yaw)
sleep(5)

t.land()

