from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.5)
sleep(10)

t.fly(FlightVector(1.00, 0.00, 0.00))
sleep(10)

t.hover(1.0)
sleep(10)

t.fly(FlightVector(-1.00, 0.00, 0.00))
sleep(10)

t.land()

