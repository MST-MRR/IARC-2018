from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0)
sleep(5)

in_air_yaw = t.get_yaw_deg()

if(not (-5.0 <= in_air_yaw <= 5.0)):
    t.hover(desired_angle=0)

in_air_yaw = t.get_yaw_deg()

# t.hover(desired_angle=180.0)
# sleep(1)

t.fly(FlightVector(-1.00, 0.00, 0.00))
sleep(10)

t.hover(desired_angle=0)
sleep(1)

t.fly(FlightVector(1.00, 0.00, 0.00))
sleep(10)

t.land()

