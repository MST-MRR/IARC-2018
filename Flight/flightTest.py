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
