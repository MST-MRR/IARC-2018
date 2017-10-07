import dronekit, PID, time, math, os
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)


print("\n Connected")

RC_MAX = 2006
RC_MIN = 982

def test_PWN(setpoint, pitchAngle, rollAngle, yawAngle):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        os.system("clear")
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
    rollPID = PID.PID(10.0, 0.0, 15.0)
    rollPID.SetPoint = rollAngle
    rollPWM = getPitchPWM(rollAngle)

    #Initialize Yaw PID Controller
    yawPID = PID.PID(10.0 , 1.5, 15.0)
    yawPID.SetPoint = yawAngle
    yawPWM = 1494

    #Circle Stuffs
    #yawVPID = PID.PID(10 ,0 ,15)
    #yawVPID.SetPoint = 2 * 3.1415926 / timeForCircle
    #yawVPWM = 1494

    #Initialize Throttle PID Controller
    pid = PID.PID(10.0, 0.0, 0.0)
    pid.SetPoint = setpoint
    PWM = RC_MIN
    vehicle.channels.overrides['3'] = PWM

    while True:
        try:
            height = vehicle.location.global_relative_frame.alt

            if height > 3:
                pid.SetPoint = 0
            time.sleep(.1)
            currentAltVel = vehicle.velocity[2]
            pid.update(currentAltVel)
            PWM += pid.output
            vehicle.channels.overrides['3'] = PWM
            print("Update throt: %s" % PWM)
            print("Alt velocity: %s" % currentAltVel)
            print("Velocites: %s " % vehicle.velocity)
            print("Height: %s " % height)
            

            #Wait until the drone is at a good height to change direction
            if (currentAltVel > setpoint / 2):
                '''     
                vehiclePitch = vehicle.velocity[0]            #Get drones pitch velocity
                pitchPID.update(vehiclePitch)                 #Update PID with current pitch          
                pitchPWM -= pitchPID.output                   #Set PWM to desired PWM from PID
                vehicle.channels.overrides['2'] = pitchPWM    #Send signal to drone
                '''
                '''
                vehicleRoll = vehicle.velocity[1]             #Get drones roll velocity
                rollPID.update(vehicleRoll)                   #Update PID with current roll 
                rollPWM += rollPID.output                     #Set PWM to desired PWM from PID                                 
                vehicle.channels.overrides['1'] = rollPWM     #Send signal to drone
                '''
                
                vehicleYaw = vehicle.attitude.yaw             #Get drones yaw angle
                yawPID.update(vehicleYaw)                     #Update PID with current yaw
                yawPWM += yawPID.output                       #Set PWM to desired PWM from PID                            
                vehicle.channels.overrides['4'] = yawPWM      #Send signal to drone   
                

                #print("Desired pitch: %s" % pitchAngle)       #Output data
                #print("Actual pitch:  %s" % vehiclePitch)
                #print("Desired roll:  %s" % rollAngle)
                #print("Actual roll:   %s" % vehicleRoll)
                print("Desired yaw:   %s" % math.degrees(yawAngle))
                print("Actual yaw:    %s" % math.degrees(vehicleYaw))
            os.system('clear')
            
                
        except KeyboardInterrupt:
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(5.0)

    vehicle.channels.overrides['3'] = RC_MIN
    vehicle.close()

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def getPitchPWM(speed):
    value = 2*(256*speed+747)
    if value > RC_MAX:
        return RC_MAX
    elif value < RC_MIN:
        return RC_MIN
    
    return value

def getRollPWM(angle):
    return (((512*angle)/45) + 1494)

# Alt, Pitch, Roll, Yaw
test_PWN(0.3, 0.0, 0.0, 0.0)
