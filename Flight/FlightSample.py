from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep, time
from os import system

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(5)

t.hover(3, 90)
sleep(5)
t.hover(None, 0.0)

# t.pitch_forward_backward(5, 0.5)
# if not t.pitch_forward_backward
# print("Done moving forward")

# sleep(10)

t.land()