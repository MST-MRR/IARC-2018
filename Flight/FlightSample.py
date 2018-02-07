from AutonomousFlight import FlightVector
from ATC import Tower
from time import sleep, time
from os import system

t = Tower()

t.initialize()

t.takeoff(2.0)
sleep(2)

while True:
    print "Going left"
    t.move_sideways(5, -0.5)
    while t.side_stop_time > time():
        sleep(0.00)
    t.hover()
    sleep(5)
    print "Going backwards"
    t.pitch_forward_backward(5, -0.5)
    while t.forward_stop_time > time():
        sleep(0.00)
    t.hover()
    sleep(5)
    print "Going right"
    t.move_sideways(5, 0.5)
    while t.side_stop_time > time():
        sleep(0.00)
    t.hover()
    sleep(5) 
    print "Going forwards"   
    t.pitch_forward_backward(5, 0.5)
    while t.forward_stop_time > time():
        sleep(0.00)
    t.hover()
    sleep(5)
    
sleep(10)
