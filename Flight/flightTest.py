import dronekit
import PID
import math
import os
import time
import RPi.GPIO as GPIO
from dronekit import VehicleMode
import Sonar
# from graph_class import Graph as PID_Graph

MAV_SENSOR_ROTATION_PITCH_270 = 25
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
PID_UPDATE_TIME = 0.00
PI = math.pi

SIM = "tcp:127.0.0.1:5762"
USB = "/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

MIN_SONAR_DISTANCE = 3
MAX_SONAR_DISTANCE = 4000
TRIG_PIN = 2
ECHO_PIN = 3

downward_sonar = Sonar.Sonar(TRIG_PIN, ECHO_PIN)

vehicle = dronekit.connect(SIM, wait_ready=True)

print("\n Connected")

def send_distance_message(distance_to_ground):
    print("Distance: %f" % distance_to_ground)
    message = vehicle.message_factory.distance_sensor_encode(
        0,                                             # time since system boot, not used
        MIN_SONAR_DISTANCE,                            # min distance cm
        MAX_SONAR_DISTANCE,                            # max distance cm
        distance_to_ground,                            # current distance, must be int
        0,                                             # type = laser
        0,                                             # onboard id, not used
        MAV_SENSOR_ROTATION_PITCH_270,                 # Downward facing range sensor.
        0                                              # covariance, not used
    )
    vehicle.send_mavlink(message)
    vehicle.commands.upload()
    

def test_flight(desired_speed, desired_alt, desired_pitch_velocity,
                desired_roll_velocity, desired_yaw_angle):
    
    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(1.0)

    # Initialize Pitch Pid Controller
    # PitchPID = PID.PID(PITCH_P, PITCH_I, PITCH_D)
    # PitchPID.SetPoint = desired_pitch_velocity
    # PitchPID.setSampleTime(PID_UPDATE_TIME)

    # PitchPID = flightControl.PIDController(desired_pitch_velocity, PITCH_MID,
    #                                        PID_UPDATE_TIME, PITCH_P, PITCH_I,
    #                                        PITCH_D)
    #PitchPWM = get_pitch_pwm(desired_pitch_velocity)

    #pitch_graph = PID_Graph(desired_pitch_velocity, "Pitch")

    # Initialize Roll PID Controller
    # RollPID = PID.PID(ROLL_P, ROLL_I, ROLL_D)
    # RollPID.SetPoint = desired_roll_velocity
    # RollPID.setSampleTime(PID_UPDATE_TIME)
    # RollPWM = ROLL_MID
    # RollPID = flightControl.PIDController(desired_roll_velocity, ROLL_MID,
    #                                       PID_UPDATE_TIME, ROLL_P, ROLL_I,
    #                                       ROLL_D)
    # RollPWM = get_pitch_pwm(desired_roll_velocity)

    #roll_graph = PID_Graph(desired_roll_velocity, "Roll")

    # Initialize Yaw PID Controller
    #desired_yaw_angle = get_yaw_radians(desired_yaw_angle)
    # YawPID = PID.PID(YAW_P , YAW_I, YAW_D)
    # YawPID.SetPoint = desired_yaw_angle
    # YawPID.setSampleTime(PID_UPDATE_TIME)
    # YawPID = flightControl.PIDController(desired_yaw_angle, YAW_MID,
    #                                      PID_UPDATE_TIME, YAW_P, YAW_I, YAW_D)
    # YawPWM = YAW_MID

    # yaw_graph = PID_Graph(desired_yaw_angle, "Yaw")

    # Circle Stuffs
    # yawVPID = PID.PID(10 ,0 ,15)
    # yawVPID.SetPoint = 2 * 3.1415926 / timeForCircle
    # yawVPWM = 1494

    # Initialize Throttle PID Controller
    ThrottlePID = PID.PID(THROTTLE_P, THROTTLE_I, THROTTLE_D)
    ThrottlePID.SetPoint = desired_speed
    ThrottlePID.setSampleTime(PID_UPDATE_TIME)
    ThrottlePWM = THRUST_LOW
    vehicle.channels.overrides[THROTTLE_CHANNEL] = ThrottlePWM
    #throttle_graph = PID_Graph(desired_speed, "Throttle")
    #graphs_list = [roll_graph, pitch_graph, throttle_graph, yaw_graph]
    while True:
        try:
            time.sleep(.1)
            distance_to_ground = downward_sonar.get_distance()
            if(distance_to_ground < 5):
                send_distance_message(5)
            else:
                send_distance_message(distance_to_ground)
            current_alt = vehicle.location.global_relative_frame.alt
            
            if current_alt > (desired_alt - 0.4):
                ThrottlePID.SetPoint = 0
            else:
                if current_alt >= (desired_alt + 0.05):
                    ThrottlePID.SetPoint = -0.3
                else: 
                    if current_alt <= (desired_alt - 0.05):
                        ThrottlePID.SetPoint = 0.3

            ThrottlePID.update(vehicle.velocity[2])
            ThrottlePWM += ThrottlePID.output
            vehicle.channels.overrides[THROTTLE_CHANNEL] = ThrottlePWM
            # throttle_graph.update(vehicle.velocity[2])
            print("Desired Alt: %s" % desired_alt)
            print("Alt: %s" % current_alt)

            # Wait until the drone is at a good height to change direction
            if (current_alt > desired_alt / 2):
                
                '''
                # Get drones pitch velocity
                current_pitch_velocity = -vehicle.velocity[0]
                # Update vehicles current pitch with new pitch
                PitchPID.update(current_pitch_velocity)
                PitchPWM += PitchPID.output
                vehicle.channels.overrides[PITCH_CHANNEL] = PitchPWM
                # Update graphs
                '''
                # Get drones roll velocity
                current_roll_velocity = vehicle.velocity[1]
                # update vehicles current pitch with new roll
                # RollPID.update(current_roll_velocity)
                # RollPWM += RollPID.output
                # vehicle.channels.overrides[ROLL_CHANNEL] = RollPWM
                # update graphs
                #roll_graph.update(current_roll_velocity)
                '''

                # Get drones yaw angle
                current_yaw_angle = vehicle.attitude.yaw
                # update vehicles current yaw channel with new yaw
                YawPID.update(current_yaw_angle)
                YawPWM += YawPID.output
                vehicle.channels.overrides[YAW_CHANNEL] = YawPWM
                # update graphs
                yaw_graph.update(current_yaw_angle)

                # Output data
                print("Desired pitch: %s" % PitchPID.SetPoint)
                print("Actual pitch:  %s" % current_pitch_velocity)
                # print("Desired roll:  %s" % desired_roll_velocity)
                # print("Actual roll:   %s" % current_roll_velocity)
                print("Desired yaw:   %s" % math.degrees(desired_yaw_angle))
                print("Actual yaw:    %s" % math.degrees(current_yaw_angle))
                '''
                os.system('clear')
                
        except KeyboardInterrupt:
            # test_Roll(RollPID)
            #test_forwards(desired_alt)

            # for x in range(0, 1):
            #     test_roll_alt(ThrottlePID,ThrottlePWM, RollPID, x)
            shutdown(vehicle)
            #display_graphs(graphs_list)
            
            break
    return


def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)


def get_pitch_pwm(angle):
    return (((512*angle)/5) + 1494)


def get_yaw_radians(angle):
    if angle < 180.0:
        if angle <= 0.0:
            angle = 0.1
        return math.radians(angle)
    else:
        if angle >= 180.0:
            angle = 179.00
            return math.radians(angle)
        return math.radians(angle-180) - PI


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
    drone.channels.overrides[THROTTLE_CHANNEL] = THRUST_LOW
    drone.mode = VehicleMode("LAND")
    drone.close()


def test_Roll(rollPID):
    time_end = time.time()+10 
    rollPID.SetPoint = 0.3
    rollPWM = 1495
    vels = [0.3, -0.3, -0.3, 0.3]
    for x in range (0,4):
        time_end = time.time()+10
        rollPID.SetPoint = vels [x]
        while time.time() < time_end:
            roll_vel = vehicle.velocity [1]
            rollPID.update(roll_vel)
            rollPWM += rollPID.output
            print(rollPWM)
            vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM
            print("Desired roll:  %s" % vels[x])
            print("Actual roll:   %s" % roll_vel)
            time.sleep(0.1)   

def test_forwards(highest_alt):
    alts = [0.3, highest_alt]
    vels = [-0.3, 0.3]
    PitchPID = PID.PID(PITCH_P, PITCH_I, PITCH_D)
    ThrottlePID = PID.PID(THROTTLE_P, THROTTLE_I, THROTTLE_D)    
    PitchPID.SetPoint = vels[1]*(-1.0)
    curr_alt = vehicle.location.global_relative_frame.alt
    for x in range(0, 2):
        ThrottlePID.SetPoint = 0
        PitchPWM = PITCH_MID
        ThrottlePWM = PITCH_MID
        time_end = time.time()+3
        while abs(curr_alt - alts[x]) > 0.1 and time.time()<time_end:
            time.sleep(0.1)
            current_throttle_velocity = vehicle.velocity[2]
            current_pitch_velocity = vehicle.velocity[0]
            PitchPID.update(current_pitch_velocity) 
            PitchPWM -= PitchPID.output
            ThrottlePID.update(current_throttle_velocity)
            ThrottlePWM += ThrottlePID.output
            vehicle.channels.overrides[PITCH_CHANNEL] = PitchPWM
            vehicle.channels.overrides[THROTTLE_CHANNEL] = ThrottlePWM
            curr_alt = vehicle.location.global_relative_frame.alt
            print("Desired Alt: %s" % alts[x])
            print("Current Alt: %s" % curr_alt)
            print("Desired throt: %s" % ThrottlePID.SetPoint)
            print("Current throt: %s" % vehicle.velocity[2])
            print("Desired pitch: %s" % PitchPID.SetPoint)
            print("Current pitch %s" % vehicle.velocity[0])
            print(" ")
    pass

def test_roll_alt(throttlePID, throttlePWM, rollPID, direction):
    if direction == 0:
        rollPID.setpoint = -0.3
    else:
        rollPID.setpoint = 0.3
    rollPWM = 1495
    vels = [-0.3, 0.3]
    alts = [0.3,3]
    current_alt = vehicle.location.global_relative_frame.alt
    for x in range (0,2):
        throttlePID.SetPoint = vels[x]
        while abs(current_alt -alts[x]) >= 0.1:
            current_throttle_velocity = vehicle.velocity [2]
            current_roll_velocity = vehicle.velocity [0]
            throttlePID.update(current_throttle_velocity)
            throttlePWM += throttlePID.output
            vehicle.channels.overrides[THROTTLE_CHANNEL] = throttlePWM
            current_roll_velocity = vehicle.velocity [1]
            rollPID.update(current_roll_velocity)
            rollPWM += rollPID.output
            vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM
            print("Desired Alt: %s" % alts[x])
            print("Actual Alt: %s" % current_alt)
            time.sleep(0.1)
            current_alt = vehicle.location.global_relative_frame.alt

# Alt, Desired alt, Pitch, Roll, Yaw
# Velocity, Meter, velocity, velocity, angle
test_flight(0.3, 1.0, 0.0, 0.0, 0.0)
