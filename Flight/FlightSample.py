from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(10)

t.hover(5, 150)
sleep(10)
print t.get_yaw_deg()

t.land()

