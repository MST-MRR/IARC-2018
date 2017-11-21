from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0, desired_angle=0)
sleep(5)

t.fly(FlightVector(1.00, 0, 0))

t.hover()

t.land()

