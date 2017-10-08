import dronekit, PID, time, math, os, numpy as np, matplotlib.pyplot as plt
from dronekit import VehicleMode
from scipy.interpolate import spline

YAW_MID = 1494
PITCH_MID = 1494
ROLL_MID = 1494
THRUST_LOW = 986.0
PITCH_P = 10
PITCH_I = 0
PITCH_D = 15
ROLL_P = 10
ROLL_I = 0
ROLL_D = 15
YAW_P = 2
YAW_I = 0
YAW_D = 8
THROTTLE_P = 15.0
THROTTLE_I = 0.0
THROTTLE_D = 10.0
ROLL_CHANNEL = '1'
PITCH_CHANNEL = '2'
THROTTLE_CHANNEL = '3'
YAW_CHANNEL = '4'
PID_UPDATE_TIME = 0.0
PI = 3.14159265359

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)

print("\n Connected")

def test_PWM(desiredSpeed, desiredAlt, desiredPitchVel, desiredRollVel, desiredYawAng):
    acutal_values_list = []
    time_list = []
    settpoint_list = []
    time_count = 0

    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(5.0)


    #Initialize Pitch Pid Controller
    pitchPID = PID.PID(PITCH_P, PITCH_I, PITCH_D)
    pitchPID.SetPoint = desiredPitchVel
    pitchPID.setSampleTime(PID_UPDATE_TIME)
    pitchPWM = getPitchPWM(desiredPitchVel)
    
    
    #Initialize Roll PID Controller
    rollPID = PID.PID(ROLL_P, ROLL_I, ROLL_D)
    rollPID.SetPoint = desiredRollVel
    rollPID.setSampleTime(PID_UPDATE_TIME)
    rollPWM = getPitchPWM(desiredRollVel)

    
    #Initialize Yaw PID Controller
    desiredYawAng = getYawRad(desiredYawAng)
    yawPID = PID.PID(YAW_P , YAW_I, YAW_D)
    yawPID.SetPoint = desiredYawAng
    yawPID.setSampleTime(PID_UPDATE_TIME)
    yawPWM = YAW_MID

    #Circle Stuffs
    #yawVPID = PID.PID(10 ,0 ,15)
    #yawVPID.SetPoint = 2 * 3.1415926 / timeForCircle
    #yawVPWM = 1494

    #Initialize Throttle PID Controller
    throttlePID = PID.PID(THROTTLE_P, THROTTLE_I, THROTTLE_D)
    throttlePID.SetPoint = desiredSpeed
    throttlePID.setSampleTime(PID_UPDATE_TIME)
    throttlePWM = THRUST_LOW
    vehicle.channels.overrides[THROTTLE_CHANNEL] = throttlePWM

    while True:
        try:
            time.sleep(.1)
            currentAlt = vehicle.location.global_relative_frame.alt
            if currentAlt > 3:
                throttlePID.SetPoint = 0
            throttlePID.update(vehicle.velocity[2])
            throttlePWM += throttlePID.output
            vehicle.channels.overrides[THROTTLE_CHANNEL] = throttlePWM
            print("Update throt: %s" % throttlePWM)
            print("Alt: %s" % currentAlt)
            
            #Wait until the drone is at a good height to change direction
            if (currentAlt > desiredAlt / 2):    
                     
                
                #currentPitchVel = vehicle.velocity[0]                 #Get drones pitch velocity
                #pitchPID.update(currentPitchVel)                      #Update PID with current pitch          
                #pitchPWM -= pitchPID.output                           #Set PWM to desired PWM from PID
                #vehicle.channels.overrides[PITCH_CHANNEL] = pitchPWM  #Send signal to drone

                currentRollVel = vehicle.velocity[1]                  #Get drones roll velocity
                rollPID.update(currentRollVel)                        #Update PID with current roll 
                rollPWM += rollPID.output                             #Set PWM to desired PWM from PID                                 
                vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM    #Send signal to drone
                
                #currentYawAng = vehicle.attitude.yaw                  #Get drones yaw angle
                #yawPID.update(currentYawAng)                          #Update PID with current yaw
                #yawPWM += yawPID.output                               #Set PWM to desired PWM from PID                            
                #vehicle.channels.overrides[YAW_CHANNEL] = yawPWM      #Send signal to drone   

                #print("Desired pitch: %s" % desiredPitchVel)          #Output data
                #print("Actual pitch:  %s" % currentPitchVel)
                print("Desired roll:  %s" % desiredRollVel)
                print("Actual roll:   %s" % currentRollVel)
                #print("Desired yaw:   %s" % math.degrees(desiredYawAng))
                #print("Actual yaw:    %s" % math.degrees(currentYawAng))
                time_count+=1
                acutal_values_list.append(currentRollVel)
                time_list.append(time_count)
                settpoint_list.append(rollPID.SetPoint)
                os.system('clear')
                
        except KeyboardInterrupt:
            time_sm = np.array(time_list)
            time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
            feedback_smooth = spline(time_list, acutal_values_list, time_smooth)

            plt.plot(time_smooth, feedback_smooth)
            plt.plot(time_list, settpoint_list)
            plt.xlim((0, time_count))
            plt.ylim((min(acutal_values_list)-0.5, max(acutal_values_list)+0.5))
            plt.xlabel('time (s)')
            plt.ylabel('PID (PV)')
            plt.title('Roll PID')


            plt.grid(True)
            plt.show()
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(5.0)

    vehicle.channels.overrides[THROTTLE_CHANNEL] = THRUST_LOW
    vehicle.close()

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def getPitchPWM(angle):
    return  (((512*angle)/5) + 1494)

def getRollPWM(angle):
    return (((512*angle)/45) + 1494)

def getYawRad(angle):
    angRad = 0
    if angle < 180:
        angRad = math.radians(angle)
    else:
        angRad = math.radians(angle-180) -  PI
    return angRad

def getBetterYaw(yaw):
    yawDeg = 0
    if (yaw < PI) and (yaw > 0):
        yawDeg = math.degrees(yaw)
    if (yaw > -PI) and (yaw < 0):
        yawDeg = math.degrees(yaw) + 360
    return yawDeg
'''
for x in range(-314, 314):
    print(getBetterYaw(float(x)/100.0))
'''

#Alt, Desired alt, Pitch, Roll, Yaw
#Velocity, Meter, velocity, velocity, angle
test_PWM(0.5, 3, 0.33, 0.3, 45.0)

