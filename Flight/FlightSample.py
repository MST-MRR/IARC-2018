from AutonomousFlight import FlightVector
from ATC import Tower

t = Tower()

t.initialize()

t.takeoff(1.0)

t.hover()

# t.fly(FlightVector(0.33, 0, 0)
