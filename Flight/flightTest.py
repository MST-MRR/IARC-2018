import dronekit, PID, time
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)


print("\n Connected")

def test_PWN(setpoint, yaw):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waititng...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(5.0)

    attitudePid = PID.PID(0.5, 0.1, 0.1)
    attitudePid.SetPoint = setpoint

    yawPid = PID.PID(0.5, 0.1, 0.1)
    yawPid.SetPoint = yaw

    PWM = getAttitudePWM(setpoint)
    print(PWM)

    vehicle.channels.overrides['3'] = PWM
    while True:
        try:
            time.sleep(1.0)
            currentAlt = vehicle.location.global_relative_frame.alt
            attitudePid.update(currentAlt)
            updateThrot = getAttitudePWM(attitudePid.output)
            vehicle.channels.overrides['3'] = updateThrot
            print("Update throt: %s" % updateThrot)
            print("Alt: %s" % currentAlt)
            
	    #currentYaw = vehicle.attitude.yaw
	    #yawPid.update(currentYaw)
	    #updateYaw = getYawPWM(yawPid.output)
	    vehicle.channels.overrides['4'] = yaw
	    #print("Update yaw: %s" % updateYaw)
	    #print("Yaw: %s" % currentYaw)
	    
            
        except KeyboardInterrupt:
            #PWMPitchAngle = getPitchPWM(-angle)
            #PWMRollAngle = getRollPWM(-angle)
            #PWMYawAngle = getYawPWM(angle)

            #vehicle.channels.overrides['2'] = PWMPitchAngle
            #vehicle.channels.overrides['1'] = PWMRollAngle
            #vehicle.channels.overrides['4'] = PWMYawAngle
            time.sleep(10)
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(10.0)

    vehicle.channels.overrides['3'] = 986.0
    vehicle.close()

    return

def getAttitudePWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)


def getPitchPWM(angle):
    return (512 * angle)/5+1494

def getRollPWM(angle):
    return (512 * angle)/5+1494 

def getYawPWM(angle):
    return (512 * angle)/5+1494

test_PWN(5.0, 5)
