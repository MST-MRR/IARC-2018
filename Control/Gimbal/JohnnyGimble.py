#Roger Downs and Alex Tariah
import RPi.GPIO as GPIO
import time

#Previous Duty Cycle is used to hold the previous value so we can calculate distance
previousAngle = 0.1
#This is the frequency of the Gimbal
FREQUENCY = 50

#Number to divide angle by to convert to duty cycle
ANGLETODUTYCONVERSIONFACTOR = 15

#Delay to allow servo to move (0.3s to move from 0 to 180 degrees)
MOVEMENTDELAY = 0.3


#Set Up
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)
pitch = GPIO.PWM(14, 100)
pitch.start(5)
pitch.ChangeFrequency(FREQUENCY) 

#Calculates the time to move from the Previous Duty Cycle to the One Inputted
def timeToMove(currentDutyCycle, desiredDutyCycle):
    distance = desiredDutyCycle - currentDutyCycle
    if distance < 0:
        distance *= -1
    timeDelay = (distance * MOVEMENTDELAY)
    timeDelay /= 180
    if timeDelay < 0.1:
        timeDelay = 0.1
    return timeDelay


def moveCamera(desiredAngle, pwm):
    global previousAngle
    #Full Left = 0 degrees = 0.1 duty cycle
    #Middle = 90 degrees = 6 duty cycle
    #Full Right = 180 degrees = 12 duty cycle
    #Desired angle can not be 0 otherwise the servo spergs out, so sets to 0.1 instead which is true 0 on servo
    if desiredAngle == 0:
        dutyCycle = 0.1
    else:
        #Calulates the duty cycle as desiredAngle / 
        dutyCycle = float(desiredAngle)/ float(ANGLETODUTYCONVERSIONFACTOR)
    sleepytime = timeToMove(previousAngle, desiredAngle)
    pwm.ChangeDutyCycle(dutyCycle)
    time.sleep(sleepytime)
    previousAngle = desiredAngle


#Moves Servo to 0.1 and sets the Previous value to 0 so we have a consistant starting value for distance calculations
moveCamera(previousAngle, pitch)
time.sleep(MOVEMENTDELAY)

try:
    moveCamera(180, pitch)
    moveCamera(45, pitch)

    pitch.stop()
    GPIO.cleanup()
except KeyboardInterrupt:
    pitch.stop()
    GPIO.cleanup()