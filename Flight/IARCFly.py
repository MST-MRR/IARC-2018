from AutonomousFlight import FlightVector
from ATC import Tower
import time
import math

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
VERTICAL_ROOMBA_POSITION = 1.0 / 3.0

def InitializeScreenScraping ( ) :
    # Some screen scraping stuff here


def InitializeScreenScraping ( ) :
    # Some screen scraping stuff here
    return


def InitializeConnection ( ) :
    t = Tower ( )
    return t

def InitializeDrone ( t ) :
    t . initialize ( )
    print ( 'Has Initialized' )

def InitialTakeoff ( t ) :
    t . takeoff ( NORMAL_FLIGHT_HEIGHT )
    t . hover ( NORMAL_FLIGHT_HEIGHT )

def InitialFlyToMiddle ( t ) :
    t . fly ( FlightVector ( NON_TRACK_LATERAL_SPEED , 0 , 0 ) )
    TimeToTravel = FLY_TO_MIDDLE_DIST / NON_TRACK_LATERAL_SPEED
    time . sleep ( TimeToTravel / SIMULATION_SPEED_FACTOR )

def ReadyHover ( t ) :
    DroneHover ( t , NORMAL_FLIGHT_HEIGHT )

def DroneHover ( t , FlightHeight ) :
    t . hover ( FlightHeight , desired_angle=0.01 )

def CalcDegreesFromBestOrientation ( Roomba ) :
    if Roomba . Direction > REVERSE_DEGREES :
        RoombaOrientation = math . abs ( CIRCLE_DEGREES - Roomba . Direction )
    else :
        RoombaOrientation = math . abs ( Roomba . Direction )

def FindBestOrientedRoomba ( Roombas ) :
    BestOrientation = REVERSE_DEGREES
    for Roomba in Roombas :
        RoombaOrientation = CalcDegreesFromBestOrientation ( Roomba )
        
        if RoombaOrientation < BestOrientation :
            BestOrientation = RoombaOrientation
            BestRoomba = Roomba

def ScreenScrape ( WindowName ) :
    ImageWindow = GetWindow ( WindowName )
    Image = Scrape ( ImageWindow )
    return Image

def CalcDesiredXPosition ( Image ) :
    DesiredXPosition = Image . Width * HORIZONTAL_ROOMBA_POSITION
    return DesiredXPosition

def CalcDesiredYPosition ( Image ) :
    DesiredYPosition = Image . Height * VERTICAL_ROOMBA_POSITION
    return DesiredYPosition

def CalcDroneXMovement ( Roomba , DesiredYPosition ) :
    XMovement = NO_SPEED
    if Roomba . Y < DesiredYPosition :
        XMovement = TRACK_LATERAL_SPEED
    else :
        XMovement = - TRACK_LATERAL_SPEED
    return XMovement

def CalcDroneYMovement ( Roomba , DesiredXPosition ) :
    YMovement = NO_SPEED
    if Roomba . X < DesiredXPosition :
        YMovement = - TRACK_LATERAL_SPEED
    else :
        YMovement = TRACK_LATERAL_SPEED
    return YMovement

def MoveToFollowRoomba ( Roomba , Image , t ) :
    DesiredXPosition = CalcDesiredXPosition ( Image )
    DesiredYPosition = CalcDesiredYPosition ( Image )
    XMovement = CalcDroneXMovement ( Roomba , DesiredYPosition )
    YMovement = CalcDroneYMovement ( Roomba , DesiredXPosition )
    DroneFlightVector = FlightVector ( XMovement , YMovement , 0 )
    t . fly ( DroneFlightVector )
    

############## The main program ##################

# Do neccesary initializations
InitializeScreenScraping ( )
t = InitializeConnection ( )
InitializeDrone ( t )

# Take off
InitialTakeoff ( t )

# Fly to middle of field
InitialFlyToMiddle ( t )

# Stop
ReadyHover ( t )


# At this point, we expect the roombas to start moving ..

# Now chase after roomba whose orientation is closest to drone orientation in image
# We will eventually change this infinite loop to check for out of bounds
while ( True == True ) :
    Image = ScreenScrape ( GAZEBO_IMAGE_WINDOW_NAME )
    
    # Not sure of Vision's api.  Hopefully its something close to this.
    DroneRoombaDetector = RoombaDetector ( )
    TargetRoombas = DroneRoombaDetector . detect ( Image )
    TargetRoomba = FindBestOrientedRoomba ( TargetRoombas )
    
    # Now set drone to move to correct position
    MoveToFollowRoomba ( TargetRoomba , Image , t )
