from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep, time
from os import system

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(2)

while True:
    t.move_sideways(10, -0.5)
    print "Moving Left"
    while(t.side_stop_time > time()):
        sleep(0.00)
    t.hover()
    sleep(5)
    t.pitch_forward_backward(10, -0.5)
    print "Moving Backward"
    while(t.forward_stop_time > time()):
        sleep(0.00)
    t.hover()
    sleep(5)
    t.move_sideways(10, 0.5)
    print "Moving Right"
    while(t.side_stop_time > time()):
        sleep(0.00)
    t.hover()
    sleep(5)
    t.pitch_velocity_based(10, 0.5)
    print "Moving Forward"
    while(t.forward_stop_time > time()):
        sleep(0.00)
    print "Hovering"
    t.hover()
    sleep(5)