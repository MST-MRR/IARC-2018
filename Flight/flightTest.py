<<<<<<< HEAD
import dronekit, PID, time, math
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)

RC_MAX = 2006
RC_MIN = 982

print("\n Connected")

def test_PWN(setpoint, pitchSpeed):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(1.0)
    
    pitchPID = PID.PID(10.0, 0.00 , 14.0)
    pitchPID.SetPoint = pitchSpeed
    pitchPWM = getPitchPWM(pitchSpeed)

    pid = PID.PID(0.5, 0.1, 0.1)
    pid.SetPoint = setpoint

    PWM = getPWM(setpoint)

    vehicle.channels.overrides['3'] = PWM
    while True:
        try:
            
            time.sleep(.1)
            currentAlt = vehicle.location.global_relative_frame.alt
            pid.update(currentAlt)
            updateThrot = getPWM(pid.output)
            vehicle.channels.overrides['3'] = updateThrot
            print("Update throt: %s" % updateThrot)
            print("Alt: %s" % currentAlt)
            

        
            if (currentAlt > setpoint / 2):
                print("Set Velocity: %s" % pitchSpeed)
                vehiclePitch = vehicle.velocity[0]
                pitchPID.update(vehiclePitch)
                pitchPWM -= pitchPID.output
                vehiclePitch = pitchPWM
                vehicle.channels.overrides[2] = pitchPWM
                print("Pitch Velocity: %s" % vehicle.velocity[0])
                print("P: %s" % pitchPID.PTerm)
                print("I: %s" % pitchPID.ITerm)
                print("D: %s" % pitchPID.DTerm)
                print("PID Output: %s" % pitchPID.output)
                print("\n\n")
            
        except KeyboardInterrupt:
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(5.0)

    vehicle.channels.overrides['3'] = None
    vehicle.close()

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def getPitchPWM(speed):
    value = ((512*-speed)/10+ 1494)
    if value > RC_MAX:
        return RC_MAX
    elif value < RC_MIN:
        return RC_MIN
    
    return value

def getRollPWM(angle):
    return (((512*angle)/45) + 1494)

test_PWN(3.0, 0.57)
=======
import time
from dronekit import VehicleMode, connect
import os
from PID import PID

vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True)

MAX_THROTTLE = vehicle.parameters['RC3_MAX']
MIN_THROTTLE = vehicle.parameters['RC3_MIN']
MAX_ALT = 3
MIN_ALT = 0

vehicle.channels.overrides[3] = MIN_THROTTLE

def convert_to_rc_safe(altitude):
  rc_out =  340.0 * altitude + 986.0
  if(rc_out < MIN_THROTTLE):
    return MIN_THROTTLE
  elif(rc_out > MAX_THROTTLE):
    return MAX_THROTTLE
  return rc_out

def convert_to_rc(altitude):
  rc_out =  340.0 * altitude + 986.0
  return rc_out

def cls():
    os.system('cls' if os.name=='nt' else 'clear')


def land_vehicle():
  print("Landing vehicle...")
  vehicle.mode = VehicleMode("LAND")
  vehicle.channels.overrides[3] = MIN_THROTTLE
  print("Cleaning up...")
  vehicle.close()
  print("Done.")

P = 0.202
I = 0.09
D = 0.05

# Agressive No Oscillation
# P = 1.25
# I = 0.2329
# D = 0.15

pid_controller = PID(P, I, D)

pid_controller.SetPoint = 1.00
pid_controller.setSampleTime(0.01)

#This will make the vehicle compensate for any idiosynchracies.
desired_mode = 'LOITER'
while vehicle.mode != desired_mode:
   vehicle.mode = VehicleMode(desired_mode)
   time.sleep(0.5)

#Arm the motors.
while not vehicle.armed:
     print("Arming motors...")
     vehicle.armed = True
     time.sleep(0.5)

while True:
  try:
    pid_controller.update(vehicle.location.global_relative_frame.alt)
    output = pid_controller.output
    rc_value = convert_to_rc(pid_controller.output)
    print("Controller Out: " + str(output) + "\nRC Out: " + str(rc_value) + "\nVehicle Altitude: " + str(vehicle.location.global_relative_frame.alt) + "\nTarget Alt: " + str(pid_controller.SetPoint))
    vehicle.channels.overrides[3] = rc_value
    time.sleep(0.02)
    cls()
  except KeyboardInterrupt:
    break

land_vehicle()
>>>>>>> origin/25cent9/experimental
