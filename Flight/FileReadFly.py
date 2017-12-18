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
TEST_FILE_NAME = 'TestFile.txt'

# Ratio of image from top of image
HORIZONTAL_ROOMBA_POSITION = 0.5

# Ratio of image from left of image
VERTICAL_ROOMBA_POSITION = 1.0 / 3.0

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

def ReadFromFile ( FileName ) :
    f = open ( FileName , "r" ) 
    TempString = f . readline ( )
    f . close ( )
    return TempString

def ReadX ( InputString ) :
    arr = InputString . split ( ' ' )
    return float ( arr [ 0 ] )

def ReadY ( InputString ) :
    arr = InputString . split ( ' ' )
    return float ( arr [ 1 ] )

def MoveToFileLocation ( DesiredX , DesiredY , t ) :
    FlyX = DesiredX
    FlyY = DesiredY
    if FlyX > 1 :
        FlyX = 1
    elif FlyX < -1 :
        FlyX = -1
    if FlyY > 1 :
        FlyY = 1
    elif FlyY < -1 :
        FlyY = -1
    
    t . fly ( FlightVector ( FlyX , FlyY , 0 ) )
    

############## The main program ##################

# Do neccesary initializations
InitializeScreenScraping ( )
t = InitializeConnection ( )
InitializeDrone ( t )

# Take off
InitialTakeoff ( t )

# Stop
ReadyHover ( t )


# At this point, we expect the roombas to start moving ..

# Now chase after roomba whose orientation is closest to drone orientation in image
# We will eventually change this infinite loop to check for out of bounds
while ( True == True ) :
    
    DirectionString = ReadFromFile ( TEST_FILE_NAME )
    DesiredX = 0
    DesiredY = 0
    if len ( DirectionString ) > 0 :
        DesiredX = ReadX ( DirectionString )
        DesiredY = ReadY ( DirectionString )
    
    
    
    MoveToFileLocation ( DesiredX , -DesiredY , t )
    
    time . sleep ( 0.05 )
