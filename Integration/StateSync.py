import requests
import json
import time

class SubDivision:
    FLIGHT = 0
    VISION = 1
    COLLISION = 2

class EndPoint:
    FLIGHT = "http://localhost:5000/api/state"
    VISION = "http://localhost:5000/api/vision"
    COLLISION = "http://localhost:5000/api/collision"
    CURRENT = "http://localhost:5000/api/current"

class StateSync:

    sender = None
    last = None
    pos_x = 0
    pos_y = 0
    pos_z = 0

    def __init__(self, sender):
        self.sender = sender

    def printCoord(self):
        print("Current coordinates:")
        print("x: " + str(self.pos_x))
        print("y: " + str(self.pos_y))
        print("z: " + str(self.pos_z))
"""
    #sends messages depending on sender
    def send(self, x, y, z)
        s_time = time.time()
        if(self.sender == VISION):
            z = 0
        else:
            x = 0
            y = 0
        text = {
            "x":x,
            "y":y,
            "z":z
        }
        if(self.sender == VISION):
            r = requests.post(EndPoint.VISION, data = json.dumps(text))
        else:
            r = requests.post(EndPoint.COLLISION, data = json.dumps(text))        
        if((time.time() - s_time) > (1.0/30)):
            print("Sync too slow!")
        if(self.sender == COLLISION):
            self.last = r
            return r
"""

    #Send a message to the API as vision
    def sendVision(self, x, y, z):
        s_time = time.time()
        text = {
            "x":x,
            "y":y,
            "z":z
        }
        r = requests.post(EndPoint.VISION, data = json.dumps(text))
        if((time.time() - s_time) > (1.0/30)):
            print("Sync too slow!")

    #Send a message to the API as collision
    def sendCollision(self, z):
        s_time = time.time()
        text = {'z':z}
        r = requests.post(EndPoint.COLLISION, data = json.dumps(text))

        if((time.time() - s_time) > (1.0/30)):
            print("Sync too slow!")
        self.last = r
        return r

    #Get drone state
    def getState(self, clearBuf = False):
        s_time = time.time()
        text = {"sender":self.sender}
        if(clearBuf):
            text['clear'] = True
        r = requests.get(EndPoint.FLIGHT, data = text)
        resp_t = json.loads(r.text)
        self.pos_x += resp_t['x']
        self.pos_y += resp_t['y']
        self.pos_z = resp_t['z']
        
        if((time.time() - s_time) > (1.0/30)):
            print("Sync too slow!")

        self.last = r
        return r

    #def send(self, msg):
     #   pass

    def getLast(self):
        return requests.get(EndPoint.CURRENT)