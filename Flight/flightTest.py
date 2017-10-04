import dronekit, PID, time, math
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)


print("\n Connected")

def test_PWN(setpoint, pitchAngle, rollAngle, yawAngle):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(7.0)


    #Initialize Pitch Pid Controller
    pitchPID = PID.PID(10, 0, 15)
    pitchPID.SetPoint = pitchAngle
    pitchPWM= getPitchPWM(pitchAngle)
    
    #Initialize Roll PID Controller
    rollPID = PID.PID(10, 0, 15)
    rollPID.SetPoint = rollAngle
    rollPWM = getPitchPWM(rollAngle)

    #Initialize Yaw PID Controller
    yawPID = PID.PID(10 , 0, 15)
    yawPID.SetPoint = yawAngle
    yawPWM = 1494

    #Circle Stuffs
    #yawVPID = PID.PID(10 ,0 ,15)
    #yawVPID.SetPoint = 2 * 3.1415926 / timeForCircle
    #yawVPWM = 1494

    #Initialize Throttle PID Controller
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
            

            #Wait until the drone is at a good height to change direction
            if (currentAlt > setpoint / 2):         
                vehiclePitch = vehicle.velocity[0]            #Get drones pitch velocity
                pitchPID.update(vehiclePitch)                 #Update PID with current pitch          
                pitchPWM -= pitchPID.output                   #Set PWM to desired PWM from PID
                vehicle.channels.overrides['2'] = pitchPWM    #Send signal to drone

                vehicleRoll = vehicle.velocity[1]             #Get drones roll velocity
                rollPID.update(vehicleRoll)                   #Update PID with current roll 
                rollPWM += rollPID.output                     #Set PWM to desired PWM from PID                                 
                vehicle.channels.overrides['1'] = rollPWM     #Send signal to drone
                
                vehicleYaw = vehicle.attitude.yaw             #Get drones yaw angle
                yawPID.update(vehicleYaw)                     #Update PID with current yaw
                yawPWM += yawPID.output                       #Set PWM to desired PWM from PID                            
                vehicle.channels.overrides['4'] = yawPWM      #Send signal to drone   

                print("Desired pitch: %s" % pitchAngle)       #Output data
                print("Actual pitch:  %s" % vehiclePitch)
                print("Desired roll:  %s" % rollAngle)
                print("Actual roll:   %s" % vehicleRoll)
                print("Desired yaw:   %s" % yawAngle)
                print("Actual yaw:    %s" % vehicleYaw)
                print("")
                
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
    return  (((512*angle)/5) + 1494)

def getRollPWM(angle):
    return (((512*angle)/45) + 1494)

test_PWN(3.0, 0, 0, 1.0)
