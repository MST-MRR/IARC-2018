from ATC import Tower, VehicleStates
import ATC
import time
import numpy as np
t = Tower()

t.connected.wait()

def fly_forward(velocity, time_frame=10):
    print ("Flying Forward")
    t.fly(np.array([-velocity, 0.00, 0.00])
    time.sleep(time_frame)
    print ("Done flying")

def fly_backward(velocity, time_frame):
    print ("Flying backward")
    t.fly(np.array([velocity, 0.00, 0.00])
    time.sleep(time_frame)
    print ("Done flying")

def fly_left(velocity, time_frame):
    print ("Flying Left")
    t.fly(np.array([0.00, velocity, 0.00])
    time.sleep(time_frame)
    print ("Done flying")

def fly_right(velocity, time_frame):
    print ("Flying Right")
    t.fly(np.array([0.00, -velocity, 0.00])
    time.sleep(time_frame)
    print ("Done flying")  
    
t.takeoff(.75)
print("Taking off")
# Can put in loop and pass timeout if something needs to be done in between
t.takeoff_completed.wait()
print("Done taking off")
t.hover()
print(t.altitude)
# Place function calls here
print("Landing")
t.land()
t.land_completed.wait()
print("Drone is done")
print(velocity)
