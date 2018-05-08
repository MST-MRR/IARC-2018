from ATC import Tower, VehicleStates
import ATC
from time import sleep
t = Tower()

t.connected.wait()

t.takeoff(1.0)
print("Taking off")
# Can put in loop and pass timeout if something needs to be done in between
t.takeoff_completed.wait()
print("Done taking off")
sleep(20)
print("Landing")
t.land()
t.land_completed.wait()
print("Drone is done")
