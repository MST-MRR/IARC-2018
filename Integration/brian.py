import requests
import StateSync as Sync

#Sync.EndPoint.FLIGHT = "https://192.168.8.1:5000"

stateGuy = Sync.StateSync("flight")
sneed = Sync.StateSync("vision")

x = 1
y = 2
z = 3

while(True):
    sneed.sendVision(x,y,z)
    stateGuy.getState()
    print "got"
    x += 1
    y += 1
    z +=1 