import dronekit, PID, time, math, os
from dronekit import VehicleMode
from graph_class import Graph as PID_Graph

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

def test_PWM(desired_speed, desired_alt, desired_pitch_velocity, desired_roll_velocity, desired_yaw_angle):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(5.0)


    #Initialize Pitch Pid Controller
    PitchPID = PID.PID(PITCH_P, PITCH_I, PITCH_D)
    PitchPID.SetPoint = desired_pitch_velocity
    PitchPID.setSampleTime(PID_UPDATE_TIME)
    PitchPWM = get_pitch_pwm(desired_pitch_velocity)

    pitch_graph = PID_Graph(desired_pitch_velocity, "Pitch")
    
    #Initialize Roll PID Controller
    RollPID = PID.PID(ROLL_P, ROLL_I, ROLL_D)
    RollPID.SetPoint = desired_roll_velocity
    RollPID.setSampleTime(PID_UPDATE_TIME)
    RollPWM = get_pitch_pwm(desired_roll_velocity)

    roll_graph = PID_Graph(desired_roll_velocity, "Roll")

    
    #Initialize Yaw PID Controller
    desired_yaw_angle = get_yaw_radians(desired_yaw_angle)
    YawPID = PID.PID(YAW_P , YAW_I, YAW_D)
    YawPID.SetPoint = desired_yaw_angle
    YawPID.setSampleTime(PID_UPDATE_TIME)
    YawPWM = YAW_MID

    yaw_graph = PID_Graph(desired_yaw_angle, "Yaw")

    #Circle Stuffs
    #yawVPID = PID.PID(10 ,0 ,15)
    #yawVPID.SetPoint = 2 * 3.1415926 / timeForCircle
    #yawVPWM = 1494

    #Initialize Throttle PID Controller
    ThrottlePID = PID.PID(THROTTLE_P, THROTTLE_I, THROTTLE_D)
    ThrottlePID.SetPoint = desired_speed
    ThrottlePID.setSampleTime(PID_UPDATE_TIME)
    ThrottlePWM = THRUST_LOW
    vehicle.channels.overrides[THROTTLE_CHANNEL] = ThrottlePWM
    throttle_graph = PID_Graph(desired_speed, "Throttle")
    
    graphs_list = [roll_graph, pitch_graph, throttle_graph, yaw_graph]

    while True:
        try:
            time.sleep(.1)
            current_alt = vehicle.location.global_relative_frame.alt
            if current_alt > 3:
                ThrottlePID.SetPoint = 0
            ThrottlePID.update(vehicle.velocity[2])
            ThrottlePWM += ThrottlePID.output
            vehicle.channels.overrides[THROTTLE_CHANNEL] = ThrottlePWM
            print("Update throt: %s" % ThrottlePWM)
            print("Alt: %s" % current_alt)
            
            #Wait until the drone is at a good height to change direction
            if (current_alt > desired_alt / 2):    
                     
                
                #current_pitch_velocity = vehicle.velocity[0]               #Get drones pitch velocity
                #PitchPID.update(current_pitch_velocity)                    #Update PID with current pitch          
                #PitchPWM -= PitchPID.output                                #Set PWM to desired PWM from PID
                #vehicle.channels.overrides[PITCH_CHANNEL] = PitchPWM       #Send signal to drone

                '''
                current_roll_velocity = vehicle.velocity[1]                 #Get drones roll velocity
                RollPID.update(current_roll_velocity)                       #Update PID with current roll 
                RollPWM += RollPID.output                                   #Set PWM to desired PWM from PID                    
                roll_graph.update(current_roll_velocity)             
                vehicle.channels.overrides[ROLL_CHANNEL] = RollPWM          #Send signal to drone
                '''

                current_yaw_angle = vehicle.attitude.yaw                    #Get drones yaw angle
                YawPID.update(current_yaw_angle)                            #Update PID with current yaw
                YawPWM += YawPID.output                                     #Set PWM to desired PWM from PID                            
                vehicle.channels.overrides[YAW_CHANNEL] = YawPWM            #Send signal to drone   
                yaw_graph.update(current_yaw_angle)

                #print("Desired pitch: %s" % desired_pitch_velocity)         #Output data
                #print("Actual pitch:  %s" % current_pitch_velocity)
                #print("Desired roll:  %s" % desired_roll_velocity)
                #print("Actual roll:   %s" % current_roll_velocity)
                print("Desired yaw:   %s" % math.degrees(desired_yaw_angle))
                print("Actual yaw:    %s" % math.degrees(current_yaw_angle))
                
                os.system('clear')
                
        except KeyboardInterrupt:
            shutdown(vehicle)
            display_graphs(graphs_list)
            break

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def get_pitch_pwm(angle):
    return  (((512*angle)/5) + 1494)

def get_yaw_radians(angle):
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

def display_graphs(graphs):
    os.system("clear")
    for graph in graphs:
        try:
            graph.display()
        except ValueError:
            print("%s has no graph" % graph.title)

def shutdown(drone):
    drone.mode = VehicleMode("LAND")
    
    time.sleep(5.0)

    drone.channels.overrides[THROTTLE_CHANNEL] = THRUST_LOW
    drone.close()

'''
for x in range(-314, 314):
    print(getBetterYaw(float(x)/100.0))
'''

#Alt, Desired alt, Pitch, Roll, Yaw
#Velocity, Meter, velocity, velocity, angle
test_PWM(0.5, 3, 0, 0, 170.0)

