from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep

t = Tower()

t.initialize()

t.takeoff(1.0)

t.fly(FlightVector(0.33, 0, 0))
sleep(15)

t.hover()
sleep(5)

t.fly(FlightVector(-0.33, 0, 0))
sleep(15)

t.hover()
sleep(10)

# t.hover(1.5)
# sleep(5)

# t.fly(FlightVector(0, 0.33, 0))
# sleep(10)

# t.hover()
# sleep(5)

# t.fly(FlightVector(0, -0.33, 0))
# sleep(10)

# t.hover()
# sleep(5)

# t.hover(1.0)
# sleep(5)

# t.hover(desired_angle=45)
# sleep(2)

# t.hover(desired_angle=0.001)
# sleep(2)

# t.hover(desired_angle=-45)

# t.hover(desired_angle=0.001)
# sleep(2)

