import dronekit, PID, time, math
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)


print("\n Connected")

def test_PWN(setpoint, angle):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waititng...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(7.0)
    
    pitchPID = PID.PID(0.5, 0.1 , .5)
    pitchPID.SetPoint = angle
    pid = PID.PID(0.5, 0.1, 0.1)
    pid.SetPoint = setpoint

    PWM = getPWM(setpoint)
    print(PWM)

    vehicle.channels.overrides['3'] = PWM
    while True:
        try:
            print(vehicle.attitude.pitch)
            
            time.sleep(1.0)
            currentAlt = vehicle.location.global_relative_frame.alt
            pid.update(currentAlt)
            updateThrot = getPWM(pid.output)
            '''vehicle.channels.overrides['3'] = updateThrot
            '''
            time.sleep(1.0)
            vehiclePitch = math.degrees(vehicle.attitude.pitch)
            pitchPID.update(vehiclePitch)
            updatePitch = getPitchPWM(pitchPID.output)
            vehicle.channels.overrides['2'] = updatePitch
            vehicle.channels.overrides['3'] = updateThrot

            '''
            print("Update throt: %s" % updateThrot)
            print("Alt: %s" % currentAlt)
            '''
            print("Update Pitch: %s" % updatePitch)
            print("Vehicle Pitch: %s" % vehiclePitch)
            
        except KeyboardInterrupt:
            PWMAngle = getPitchPWM(angle)
            vehicle.channels.overrides['2'] = PWMAngle
            time.sleep(1)
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(10.0)

    vehicle.channels.overrides['3'] = 986.0
    vehicle.close()

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def getPitchPWM(angle):
    return (((256*angle)/5) + 1494)

def getRollPWM(angle):
    return (((256*angle)/5) + 1238)

test_PWN(3.0, 5.0)
