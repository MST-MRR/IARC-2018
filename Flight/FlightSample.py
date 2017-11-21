from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0, desired_angle=43.0)
sleep(5)

t.hover(desired_angle=-21.0)

t.fly(FlightVector(0.33, 0.00, 0.00))
sleep(10)

t.hover()
sleep(5)

t.fly(FlightVector(-0.33, 0.00, 0.00))
sleep(10)

t.hover()
sleep(2)

t.hover(desired_angle=-120)
sleep(5)

t.fly(FlightVector(0.33, 0.00, 0.00))
sleep(10)

t.hover()
sleep(5)

t.fly(FlightVector(-0.33, 0.00, 0.00))
sleep(10)

t.hover()
sleep(5)

t.land()

