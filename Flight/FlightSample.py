from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(10)

t.fly(FlightVector(0.5, 0.0, 0.50))
sleep(10)

t.hover(2.0)
sleep(10)

t.fly(FlightVector(-0.5, 0.0, -0.25))
sleep(5)

t.fly(FlightVector(-0.5, 0.0, -0.0))
sleep(10)

t.fly(FlightVector(0.50, 0.0, 0.30))
sleep(10)

t.land()

