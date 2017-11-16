from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0, desired_angle=45)
sleep(10)

# t.fly(FlightVector(0.33, 0, 0))
# sleep(10)

# t.hover(desired_angle=45)
# sleep(5)

# t.fly(FlightVector(-0.33, 0, 0))
# sleep(10)

t.hover(desired_angle=-120)
sleep(5)

t.hover(desired_angle=0)
sleep(5)

# t.hover()
# sleep(10)
