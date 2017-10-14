import dronekit, PID, time, math, os
from dronekit import VehicleMode
from graph_class import Graph as PID_Graph

YAW_MID = 1494
PITCH_MID = 1494
ROLL_MID = 1494
THRUST_LOW = 986.0
PITCH_P = 10.0
PITCH_I = 0.0
PITCH_D = 15.0
ROLL_P = 10.0
ROLL_I = 0.0
ROLL_D = 15.0
YAW_P = 2.0
YAW_I = 0.0
YAW_D = 9.0
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

def test_flight(desired_speed, desired_alt, desired_pitch_velocity, desired_roll_velocity, desired_yaw_angle):
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
            if current_alt > 2.5:
                ThrottlePID.SetPoint = 0
            ThrottlePID.update(vehicle.velocity[2])
            ThrottlePWM += ThrottlePID.output
            vehicle.channels.overrides[THROTTLE_CHANNEL] = ThrottlePWM
            throttle_graph.update(vehicle.velocity[2])
            print("Update throt: %s" % ThrottlePWM)
            print("Alt: %s" % current_alt)
            
            #Wait until the drone is at a good height to change direction
            if (current_alt > desired_alt / 2):    
 
                
                current_pitch_velocity = vehicle.velocity[0]               #Get drones pitch velocity
                PitchPID.update(current_pitch_velocity)                    #Update PID with current pitch          
                PitchPWM -= PitchPID.output                                #Set PWM to desired PWM from PID
                vehicle.channels.overrides[PITCH_CHANNEL] = PitchPWM       #Send signal to drone
                pitch_graph.update(current_pitch_velocity)
                
                current_roll_velocity = vehicle.velocity[1]                 #Get drones roll velocity
                RollPID.update(current_roll_velocity)                       #Update PID with current roll 
                RollPWM += RollPID.output                                   #Set PWM to desired PWM from PID                    
                roll_graph.update(current_roll_velocity)             
                vehicle.channels.overrides[ROLL_CHANNEL] = RollPWM          #Send signal to drone
                

                current_yaw_angle = vehicle.attitude.yaw                    #Get drones yaw angle
                YawPID.update(current_yaw_angle)                            #Update PID with current yaw
                YawPWM += YawPID.output                                     #Set PWM to desired PWM from PID                            
                vehicle.channels.overrides[YAW_CHANNEL] = YawPWM            #Send signal to drone   
                yaw_graph.update(current_yaw_angle)

                print("Desired pitch: %s" % desired_pitch_velocity)         #Output data
                print("Actual pitch:  %s" % current_pitch_velocity)
                print("Desired roll:  %s" % desired_roll_velocity)
                print("Actual roll:   %s" % current_roll_velocity)
                print("Desired yaw:   %s" % math.degrees(desired_yaw_angle))
                print("Actual yaw:    %s" % math.degrees(current_yaw_angle))
                
                os.system('clear')
                
        except KeyboardInterrupt:
            test_Roll(RollPID)
            shutdown(vehicle)
            display_graphs(graphs_list)
            break

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)

def get_pitch_pwm(angle):
    return  (((512*angle)/5) + 1494)

def get_yaw_radians(angle):
    if angle < 180:
        return math.radians(angle)
    else:
        return math.radians(angle-180) -  PI
    
def getBetterYaw(yaw):
    if (yaw < PI) and (yaw > 0):
        return math.degrees(yaw)
    if (yaw > -PI) and (yaw < 0):
        return math.degrees(yaw) + 360

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

def test_Roll(rollPID):
    time_end = time.time()+10 
    rollPID.SetPoint = 0.3
    rollPWM = 1495
    print("Going to the right")
    while time.time() < time_end:
         roll_vel = vehicle.velocity[1]
         rollPID.update(roll_vel) 
         rollPWM += rollPID.output
         vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM
         print("Desired roll:  0.3"  )
         print("Actual roll:   %s" % roll_vel)
         time.sleep(0.1)
    time_end = time.time()+10
    rollPID.SetPoint = -0.3
    print("Going to the original position")
    while time.time() < time_end:
         roll_vel = vehicle.velocity[1]
         rollPID.update(roll_vel) 
         rollPWM += rollPID.output
         vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM
         print("Desired roll:  -0.3"  )
         print("Actual roll:   %s" % roll_vel)
         time.sleep(0.1) 
    time_end = time.time()+10
    rollPID.SetPoint = -0.3
    print("Going to the left")              
    while time.time() < time_end:
         roll_vel = vehicle.velocity[1]
         rollPID.update(roll_vel) 
         rollPWM += rollPID.output
         vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM
         print("Desired roll:  -0.3"  )
         print("Actual roll:   %s" % roll_vel)
         time.sleep(0.1)       
    time_end = time.time()+10
    rollPID.SetPoint = 0.3
    print("Going to the original position")              
    while time.time() < time_end:
         roll_vel = vehicle.velocity[1]
         rollPID.update(roll_vel) 
         rollPWM += rollPID.output
         vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM
         print("Desired roll:  0.3"  )
         print("Actual roll:   %s" % roll_vel)
         time.sleep(0.1)           
'''
for x in range(-314, 314):
    print(getBetterYaw(float(x)/100.0))
'''

#Alt, Desired alt, Pitch, Roll, Yaw
#Velocity, Meter, velocity, velocity, angle
test_flight(0.3, 3, 0.0, 0.0, 1.0)

