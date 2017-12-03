from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(5)

t.fly(FlightVector(0.33, 0, 0))
sleep(10)

t.hover(desired_angle=-120)
sleep(5)

t.fly(FlightVector(0.33, 0, 0))
sleep(10)

t.land()

