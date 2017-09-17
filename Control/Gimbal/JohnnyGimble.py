#Roger Downs and Alex Tariah
import RPi.GPIO as GPIO
import time

#This is the frequency of the Gimbal
FREQUENCY = 50

#Number to divide angle by to convert to duty cycle
ANGLETODUTYCONVERSIONFACTOR = 15 

#Delay to allow servo to move
MOVEMENTDELAY = 0.5


#Set Up
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)
pitch = GPIO.PWM(14, 100)
pitch.start(5)
pitch.ChangeFrequency(FREQUENCY) 

#Duty Cycle = Movement 
def update(desiredAngle, pwm):
    #Full Left = 0 degrees = 1
    #Middle = 90 degrees = 6
    #Full Right = 180 degrees = 12
    if desiredAngle == 0:
        dutyCycle = 1
    else:
        dutyCycle = desiredAngle / ANGLETODUTYCONVERSIONFACTOR
    pwm.ChangeDutyCycle(dutyCycle)
    #wait while encoder value not equal to desiredAngle


try:
   while(True):
       #CHANGE SLEEP TO ENCODER VALUES ONCE SERVO HAS ENCODER
    update(0, pitch)
    time.sleep(MOVEMENTDELAY)
    update(180, pitch)
    time.sleep(MOVEMENTDELAY)
    update(90, pitch)
    time.sleep(MOVEMENTDELAY)
except KeyboardInterrupt:
    pitch.stop()
    GPIO.cleanup()