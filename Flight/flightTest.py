import dronekit
import PID
import time
import math
import os
from dronekit import VehicleMode

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


def test_PWM(desiredSpeed,
             desiredAlt,
             desiredPitchVel,
             desiredRollVel,
             desiredYawAng):
    vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)

    print("\n Connected")
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(5.0)

    # Initialize Pitch Pid Controller
    pitchPID = PID.PID(PITCH_P, PITCH_I, PITCH_D)
    pitchPID.SetPoint = desiredPitchVel
    pitchPID.setSampleTime(PID_UPDATE_TIME)
    pitchPWM = PITCH_MID

    # Initialize Roll PID Controller
    rollPID = PID.PID(ROLL_P, ROLL_I, ROLL_D)
    rollPID.SetPoint = desiredRollVel
    rollPID.setSampleTime(PID_UPDATE_TIME)
    rollPWM = ROLL_MID

    # Initialize Yaw PID Controller
    desiredYawAng = getYawRad(desiredYawAng)
    yawPID = PID.PID(YAW_P, YAW_I, YAW_D)
    yawPID.SetPoint = desiredYawAng
    yawPID.setSampleTime(PID_UPDATE_TIME)
    yawPWM = YAW_MID
    yawRevolutions = 0
    currentYawAng = 0
    previousYawAng = 0

    # Initialize Throttle PID Controller
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

            # Wait until the drone is at a good height to change direction
            if (currentAlt > desiredAlt / 2):
                currentPitchVel = vehicle.velocity[0]
                pitchPID.update(currentPitchVel)
                pitchPWM -= pitchPID.output
                vehicle.channels.overrides[PITCH_CHANNEL] = pitchPWM

                currentRollVel = vehicle.velocity[1]
                rollPID.update(currentRollVel)
                rollPWM += rollPID.output
                vehicle.channels.overrides[ROLL_CHANNEL] = rollPWM

                previousYawAng = currentYawAng
                currentYawAng = getBetterYaw(vehicle.attitude.yaw)
                currentYawAng += yawRevolutions * 360
                if (previousYawAng > 300 and currentYawAng < 60):
                    yawRevolutions += 1
                elif (previousYawAng < 60 and currentYawAng > 300):
                    yawRevolutions -= 1
                yawPID.update(currentYawAng)
                yawPWM += yawPID.output
                vehicle.channels.overrides[YAW_CHANNEL] = yawPWM

                os.clear()
                print("Desired pitch: %s" % desiredPitchVel)
                print("Actual pitch:  %s" % currentPitchVel)
                print("Desired roll:  %s" % desiredRollVel)
                print("Actual roll:   %s" % currentRollVel)
                print("Desired yaw:   %s" % math.degrees(desiredYawAng))
                print("Actual yaw:    %s" % math.degrees(currentYawAng))
                print("")

        except KeyboardInterrupt:
            break

    vehicle.mode = VehicleMode("LAND")
    time.sleep(10.0)
    vehicle.channels.overrides[THROTTLE_CHANNEL] = THRUST_LOW
    vehicle.close()
    return


def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)


def getPitchPWM(angle):
    return (((512*angle)/5) + 1494)


def getRollPWM(angle):
    return (((512*angle)/45) + 1494)


def getYawRad(angle):
    angRad = 0
    if angle < 180:
        angRad = math.radians(angle)
    else:
        angRad = math.radians(angle-180) - PI
    return angRad


def getBetterYaw(yaw):
    yawDeg = 0
    if (yaw < PI) and (yaw > 0):
        yawDeg = math.degrees(yaw)
    if (yaw > -PI) and (yaw < 0):
        yawDeg = math.degrees(yaw) + 360
    return yawDeg


for x in range(-314, 314):
    print(getBetterYaw(float(x)/100.0))

# Alt, Pitch, Roll, Yaw
test_PWM(0.5, 3, 0, 0, 45.0)
