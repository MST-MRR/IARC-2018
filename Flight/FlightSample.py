from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0)
sleep(10)

# t.fly(FlightVector(1.0, 0.0, 0.33))
# sleep(10)

t.hover(5.0)
sleep(10)

t.hover(1.0)
sleep(10)


# t.fly(FlightVector(-1.0, 0.0, -0.33))
# sleep(10)

t.land()

