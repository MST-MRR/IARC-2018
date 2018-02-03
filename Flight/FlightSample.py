from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep
from os import system

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(5)

t.pitch_forward(1)

while t.target_distance > 0:
    print t.target_distance
