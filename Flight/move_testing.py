from ATC import Tower, VehicleStates
import ATC
import time
import numpy as np
from timeit import default_timer as timer
def fly_forward(velocity, time_frame):
    print ("Flying Forward")

    t.fly(np.array([-velocity, 0.00, 0.00]))

    time.sleep(time_frame)

    print ("Done flying")



def fly_backward(velocity, time_frame):

    print ("Flying backward")

    t.fly(np.array([velocity, 0.00, 0.00]))

    time.sleep(time_frame)

    print ("Done flying")



def fly_left(velocity, time_frame):

    print ("Flying Left")

    t.fly(np.array([0.00, velocity, 0.00]))

    time.sleep(time_frame)

    print ("Done flying")



def fly_right(velocity, time_frame):

    print ("Flying Right")

    t.fly(np.array([0.00, -velocity, 0.00]))

    time.sleep(time_frame)

    print ("Done flying")  

t = Tower()

t.connected.wait()    

t.takeoff(1.3)

print("Taking off")

# Can put in loop and pass timeout if something needs to be done in between

t.takeoff_completed.wait()

print("Done taking off")

t.hover()

time.sleep(10)
#while np.abs(current_time - timer())<= 150:
#	print("Degrees from North: ", t.angle_from_goal(), "goal angle: ", t.goal_angle, "yaw_angle: ", t.drone_yaw_angle())
#print("decreasing")
#t.pid_flight_controller.target_altitude =.75
#time.sleep(8) 
	
# Place function calls here


print("Landing")

t.land()

t.land_completed.wait()

print("Drone is done")
