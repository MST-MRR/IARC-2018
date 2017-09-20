import dronekit, PID, time, math
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)


print("\n Connected")

def test_PWN(setpoint, angle):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(7.0)
    
    pitchPID = PID.PID(.6, 0.00 , .2)
    pitchPID.SetPoint = angle
    pitchPWM= getPitchPWM(angle)

    pid = PID.PID(0.5, 0.1, 0.1)
    pid.SetPoint = setpoint

    PWM = getPWM(setpoint)
    print(PWM)

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
                vehiclePitch = math.degrees(vehicle.attitude.pitch)
                pitchPID.update(vehiclePitch)
                pitchPWM += pitchPID.output
                updatePitch = pitchPWM
                vehicle.channels.overrides['2'] = updatePitch
                print("Update Pitch: %s" % updatePitch)
                print("Vehicle Pitch: %s" % vehiclePitch)
                print("P: %s" % pitchPID.PTerm)
                print("I: %s" % pitchPID.ITerm)
                print("D: %s" % pitchPID.DTerm)
                print("PID Output: %s" % pitchPID.output)
            
            
        except KeyboardInterrupt:
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(10.0)

    vehicle.channels.overrides['3'] = 986.0
    vehicle.close()

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def getPitchPWM(angle):
    return  (((256*-1*angle)/45) + 1494)

def getRollPWM(angle):
    return (((512*angle)/45) + 1494)

test_PWN(3.0, 0.4)
