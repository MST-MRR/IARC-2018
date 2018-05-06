from Realsense import Realsense
from time import sleep
from ATC import Tower, VehicleStates
import cv2
import numpy as np
import sys 
import mrrdt_vision
from mrrdt_vision.obj_detect.roomba_cnn import RoombaDetector
from timeit import default_timer as timer
from sklearn.preprocessing import normalize

CAMERA_FIELD_OF_VIEW = 64.9
SPEED_CHANGE_DIV = 0.05
X_SPEED_CHANGE_DIV = SPEED_CHANGE_DIV
Y_SPEED_CHANGE_DIV = SPEED_CHANGE_DIV
GRADIENT_SPEED_CHANGE = False

ROOMBA_CHANGE_DIV = 0.5
X_ROOMBA_CHANGE_DIV = ROOMBA_CHANGE_DIV
Y_ROOMBA_CHANGE_DIV = ROOMBA_CHANGE_DIV
ROOMBA_CHANGE_COMPE = False

ATTEMPT_LAND = True
LAND_CHANGE_SPEED = 0.6
ROOMBA_TRACKING_SPEED = 0.5
IsRoombaLastImage = False
image_height = 0
image_width = 0
hovering = False


def DroneTooFast () :
    IsTooFast = False
    DroneVelocity = t.vehicle.velocity
    if abs (DroneVelocity[0]) > ROOMBA_TRACKING_SPEED + SPEED_CHECK_PLUS :
        IsTooFast = True
    if abs(DroneVelocity[1])>ROOMBA_TRACKING_SPEED + SPEED_CHECK_PLUS:
        IsTooFast = True
    if abs(DroneVelocity[2])>ROOMBA_TRACKING_SPEED + SPEED_CHECK_PLUS:
        IsTooFast = True
    return IsTooFast

def GetTrueDroneVelocity () :
    DroneVelocity = t.vehicle.velocity
    return DroneVelocity

def GetTrueDroneVelocityXY () :
    DroneVelocity = GetTrueDroneVelocity ()
    DroneVelocityXY = DroneVelocity [ 0 : 2 ]
    return DroneVelocityXY

def GetRealSpeedXY () :
    DroneVelocity = GetTrueDroneVelocity ( )
    DroneVelocityXY = DroneVelocity [ 0 : 2 ]
    return DroneVelocityXY

def GetNewSpeed (Old , NewGoal , SpeedDiv ) :
    New = Old + ( ( NewGoal - Old ) * SpeedDiv )
    #print ( str ( New ) + ' ' + str ( NewGoal ) + ' ' + str ( Old ) )
    return New
    
def RoombaCompensate (RoombaOld , RoombaNew , SpeedDiv ) :
    New = RoombaOld + ( ( RoombaNew - RoombaOld ) ) * ROOMBA_CHANGE_DIV
    return New

def get_velocity_vector2d(start, goal, speed , XStay , YStay , RoombaPrevX , RoombaPrevY ):
    """
    @purpose:
        Takeoff and hover at an altitude of `SimpleDroneAI.TAKEOFF_HEIGHT` meters
        Note: This function will dampen the speed based on the distance we are away from the target
    @args:
        start, np.array: 2D position vector corresponding to the start point
        goal, np.array: 2D position vector corresponding to the destination point
        speed, float: speed limit in m/s
    @returns:
        A Numpy array pointed in the direction of `goal` with speed less than or equal to `speed`.
    """
    dist = np.sqrt(np.sum((goal-start)**2))
    x_vel, y_vel = min(ROOMBA_TRACKING_SPEED * 2 * 1.2 * dist/(np.sqrt(image_height * image_width)), ROOMBA_TRACKING_SPEED)*normalize((goal - start).reshape(-1, 1))
    if ROOMBA_CHANGE_COMPE == True :
        OldX = x_vel
        OldY = y_vel
        x_vel = RoombaCompensate ( RoombaPrevX , x_vel , X_ROOMBA_CHANGE_DIV )
        y_vel = RoombaCompensate ( RoombaPrevY , y_vel , Y_ROOMBA_CHANGE_DIV )
        RoombaPrevX = OldX
        RoombaPrevY = OldY
    if GRADIENT_SPEED_CHANGE == True :
        x_vel = GetNewSpeed ( XStay , x_vel , X_SPEED_CHANGE_DIV )
        y_vel = GetNewSpeed ( YStay , y_vel , Y_SPEED_CHANGE_DIV )
        XStay = x_vel
        YStay = y_vel
    return np.array([-y_vel, x_vel, 0]) , XStay , YStay , RoombaPrevX , RoombaPrevY

def follow_nearest_roomba(roombas, drone_midpoint , XStay , YStay , RoombaPrevX , RoombaPrevY ):
    """
    @purpose:
        Attempts to follow the roomba closest to the drone
    @args:
        roombas, collection of Roomba instances: A list of detected roombas
        drone_midpoint, np.array: Midpoint of the drone, we navigate relative to this point
    @returns:
    """
    
    # Uncomment when in python2
    if DroneTooFast () :
        t.hover()
    elif roombas:
        roomba_midpoints = np.asarray([roomba.center for roomba in roombas])
        target_idx = np.argmin(np.sum((roomba_midpoints-drone_midpoint)**2, axis=1))
        target = roombas[target_idx]

        velocity_vector , XStay , YStay , RoombaPrevX , RoombaPrevY = get_velocity_vector2d(
            drone_midpoint, roomba_midpoints[target_idx], ROOMBA_TRACKING_SPEED , XStay , YStay , RoombaPrevX , RoombaPrevY )

        print(velocity_vector)
        t.fly(velocity_vector)
        hovering = False

    # elif timer() - self._time_since_last_roomba >= SimpleDroneAI.MAX_LOST_TARGET_TIME:
    #     # self._tower.hover()
    #     self._prev_roomba_timer = None
    #     print ( "hovering1" )

    return XStay , YStay , RoombaPrevX , RoombaPrevY

def update(image, roombas , XStay, YStay, RoombaPrevX, RoombaPrevY):
    """
    @purpose:
        Event handler which updates relevant state variables and issues commands to the drone
    @args:
        img, np.ndarray: An image from the drone's camera.
    @returns:
    """
    image_height, image_width = image.shape[:2]
    drone_midpoint = np.asarray([image_width/2, image_height/2])
    print (RoombaPrevX, RoombaPrevY)
    XStay, YStay, RoombaPrevX, RoombaPrevY = follow_nearest_roomba(roombas, drone_midpoint, XStay, YStay, RoombaPrevX, RoombaPrevY)

    if roombas:
        roomba_midpoints = np.asarray([roomba.center for roomba in roombas])
        target_idx = np.argmin(np.sum((roomba_midpoints-drone_midpoint)**2, axis=1))
        target = roombas[target_idx]
        IsRoombaLastImage = True
    else :
        IsRoombaLastImage = False
        

    return XStay, YStay, RoombaPrevX, RoombaPrevY

with Realsense((640, 480)) as realsense:
    try:
        roomba_detector = RoombaDetector(threshold=.99)
        cv2.namedWindow("REALSENSE_STREAM_WINDOW", cv2.WINDOW_AUTOSIZE)
        t = Tower()
        # Uncomment for python2 threading
        t.connected.wait()
        print("Taking off")
        t.takeoff(1.0)
        t.takeoff_completed.wait()
        print("Done taking off")
        dronex = 0
        droney = 0
        roombaprevx = 0
        roombaprevy = 0
        while True:
            status, color, depth = realsense.next()
            if status:
                roombas = roomba_detector.detect(color)
                if roombas:
                    for roomba in roombas:
                        roomba.draw(color)
                realsense.render(color, depth)
                dronex, droney, roombaprevx, roombaprevy = update(color, roombas, dronex, droney, roombaprevx, roombaprevy)
    except KeyboardInterrupt as e:
        print('Quitting...')
