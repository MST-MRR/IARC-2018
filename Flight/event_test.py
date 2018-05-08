from ATC import Tower, VehicleStates
import ATC
import time
t = Tower()

t.connected.wait()

t.takeoff(1.5)
print("Taking off")
# Can put in loop and pass timeout if something needs to be done in between
t.takeoff_completed.wait()
print("Done taking off")
t.hover()
print(t.altitude)
print(time.time())
time.sleep(10)
print(time.time())
print("Landing")
t.land()
t.land_completed.wait()
print("Drone is done")
