from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(5)

t.fly(FlightVector(-1.0, 1.0, 1.0))
sleep(10)

t.hover(1.0)
sleep(5)

t.fly(FlightVector(-1.0, -1.0, 0.0))
sleep(10)

t.land()
