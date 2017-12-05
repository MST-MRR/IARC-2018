from AutonomousFlight import FlightVector
from ATC import Tower
import time
import math
import mrrdt_vision

NORMAL_FLIGHT_HEIGHT = 0.7
NON_TRACK_LATERAL_SPEED = 2
FLY_TO_MIDDLE_DIST = 12
DRONE_START_HEIGHT = 0
REVERSE_DEGREES = 180
CIRCLE_DEGREES = 360
GAZEBO_IMAGE_WINDOW_NAME = 'Gazebo: Image View'
TRACK_LATERAL_SPEED = 0.6
NO_SPEED = 0
SIMULATION_SPEED_FACTOR = 0.83

# Ratio of image from top of image
HORIZONTAL_ROOMBA_POSITION = 0.5

# Ratio of image from left of image
VERTICAL_ROOMBA_POSITION = 0.5


# from Lucas' Test1_v4.py file
import trollius
from trollius import From
import sys
from PIL import Image
import numpy as np

import pygazebo
import pygazebo.msg.image_stamped_pb2


# Yes, a global variable.  I don't know how else to do it. -Tanner
image = np.ndarray(shape = (0,0,0))
width = 0
height = 0
HasImage = False



@trollius.coroutine
def publish_loop():
    
    print('Connecting to get image')
    manager = yield From(pygazebo.connect()) 

    def callback(data):
        print ( 'Got mail.' )
        message = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
        global width
        global height
        global image
        width = message.image.width
        height = message.image.height
        datalist = list ( message.image.data )
        npdatalist = np.array(datalist)
        npdatalist = np.reshape(npdatalist , (height, width, 3))
        
        global HasImage
        HasImage = True

        image = npdatalist # added by Tanner

        
    
    # originally /gazebo/default/iris/iris_demo/gimbal_small_2d/tilt_link/camera/image
    subscriber = manager.subscribe('/gazebo/default/iris/iris_demo/gimbal_small_2d/tilt_link/camera/image',
                     'gazebo.msgs.ImageStamped',
                     callback)

    yield From(subscriber.wait_for_connection())
    print ( 'Entering Yield Loop' )
    while(True):
        print ( 'fire' )
        yield From(trollius.sleep(1.00))
    print(dir(manager))

import logging

logging.basicConfig()

# Deleted Trollius stuff here.  Put Trollius stuff at bottom. -Tanner

# end of code from Lucas' Test1_v4.py file -Tanner



def InitializeConnection ( ) :
    t = Tower ( )
    return t

def InitializeDrone ( t ) :
    t . initialize ( )
    print ( 'Has Initialized' )

def InitialTakeoff ( t ) :
    t . takeoff ( NORMAL_FLIGHT_HEIGHT * 2 )

def InitialFlyToMiddle ( t ) :
    t . fly ( FlightVector ( NON_TRACK_LATERAL_SPEED , 0 , 0 ) )
    TimeToTravel = FLY_TO_MIDDLE_DIST / NON_TRACK_LATERAL_SPEED
    time . sleep ( TimeToTravel / SIMULATION_SPEED_FACTOR )

def ReadyHover ( t ) :
    DroneHover ( t , NORMAL_FLIGHT_HEIGHT * 2 )

def DroneHover ( t , FlightHeight ) :
    t . hover ( FlightHeight , desired_angle=0.01 )

def CalcDistFromPointInImage ( Roomba , Width , Height ) :
    RoombaCenterX = Roomba.center[0]
    RoombaCenterY = Roomba.center[1]
    DesiredXPosition = CalcDesiredXPosition ( Width )
    DesiredYPosition = CalcDesiredYPosition ( Height )
    A = DesiredXPosition - RoombaCenterX
    B = DesiredYPosition - RoombaCenterY
    DistFromRoombaPixels = math . sqrt ( A * A + B * B )
    
    return DistFromRoombaPixels

def FindBestPositionedRoomba ( Roombas , Width , Height ) :
    BestRoomba = None
    BestImageDistFromRoomba = 99999999
    for Roomba in Roombas :
        ImageDistFromRoomba = CalcDistFromPointInImage ( Roomba , Width , Height )
        
        if ImageDistFromRoomba < BestImageDistFromRoomba :
            BestImageDistFromRoomba = ImageDistFromRoomba
            BestRoomba = Roomba
    return BestRoomba

def ScreenScrape ( WindowName ) :
    ImageWindow = GetWindow ( WindowName )
    Image = Scrape ( ImageWindow )
    return Image

def CalcDesiredXPosition ( Width ) :
    DesiredXPosition = Width * HORIZONTAL_ROOMBA_POSITION
    return DesiredXPosition

def CalcDesiredYPosition ( Height ) :
    DesiredYPosition = Height * VERTICAL_ROOMBA_POSITION
    return DesiredYPosition

def CalcDroneXMovement ( Roomba , DesiredYPosition ) :
    XMovement = NO_SPEED
    if Roomba != None :
        if Roomba . center [ 1 ] < DesiredYPosition :
            XMovement = TRACK_LATERAL_SPEED
        else :
            XMovement = - TRACK_LATERAL_SPEED
    return XMovement

def CalcDroneYMovement ( Roomba , DesiredXPosition ) :
    YMovement = NO_SPEED
    if Roomba != None :
        if Roomba . center [ 0 ] < DesiredXPosition :
            YMovement = - TRACK_LATERAL_SPEED
        else :
            YMovement = TRACK_LATERAL_SPEED
    return YMovement

def MoveToFollowRoomba ( Roomba , width , height , t ) :
    DesiredXPosition = CalcDesiredXPosition ( width )
    DesiredYPosition = CalcDesiredYPosition ( height )
    XMovement = CalcDroneXMovement ( Roomba , DesiredYPosition )
    YMovement = CalcDroneYMovement ( Roomba , DesiredXPosition )
    DroneFlightVector = FlightVector ( XMovement , YMovement , 0 )
    t . fly ( DroneFlightVector )
    

############## The main program ##################
@trollius.coroutine
def main():
    yield From(trollius.sleep(2.00))
    # Do neccesary initializations
    t = InitializeConnection ( )
    InitializeDrone ( t )
    global image

    # Take off
    InitialTakeoff ( t )

    # Fly to middle of field
    # InitialFlyToMiddle ( t )
    
    # Stop
    # ReadyHover ( t )
    # print('ReadyHover( t ) done')
    
    # At this point, we expect the roombas to start moving ..
    
    # Now chase after roomba whose orientation is closest to drone orientation in image
    # We will eventually change this infinite loop to check for out of bounds
    global width
    global height
    global HasImage
    while ( True == True ) :
        yield From(trollius.sleep(0.1)) # added by Tanner
        
        while HasImage == False :
            time . sleep ( 0.1 )
            print ( 'No Image Yet' )
       
        print('about to create roomba detector')
        DroneRoombaDetector = RoombaDetector ( )
        print('about to detect roombas')
        TargetRoombas = DroneRoombaDetector . detect ( image )
        print('about to find best roomba')
        TargetRoomba = FindBestPositionedRoomba ( TargetRoombas , width , height )
        
        # Now set drone to move to correct position
        print('about to follow best roomba')
        MoveToFollowRoomba ( TargetRoomba , width , height , t )
        
        print ( 'successful' )





tasks = []
tasks.append(trollius.Task(main()))
tasks.append(trollius.Task(publish_loop()))
loop = trollius.get_event_loop()
print('hello')
loop.run_until_complete(trollius.wait(tasks))

