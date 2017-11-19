from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0)
sleep(10)

in_air_yaw = t.get_yaw_deg()

t.hover(desired_angle=in_air_yaw - 45.0)

t.fly(FlightVector(1.00, 0.00, 0.00))
sleep(20)

t.hover()
sleep(10)

t.land()

