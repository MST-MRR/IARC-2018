#To run server:
#   gunicorn -b localhost:5000 main:api --reload
#   Binds server to localhost:5000 running the REST API in main.py,
#       Reloads if main.py or imports change

import falcon   #REST API library
import json
import DroneState as State

droneState = State.DroneState()
lastState = State.DroneState()
droneState.cleared = True

def printState():
    print ("x: " + str(droneState.x))
    print ("y: " + str(droneState.y))
    print ("z: " + str(droneState.z))
    print ("coll_msg" + str(droneState.coll_msg))

class currentHandler:
    def on_get(self, req, resp):
        text = {
            "x":lastState.x,
            "y":lastState.y,
            "z":lastState.z,
            "min_z":lastState.min_z,
            "cleared":lastState.cleared
        }
        resp.body = json.dumps(text)

class visionHandler:
    #Vision updates the drone state variables: x,y
    def on_post(self, req, resp):
        text = json.loads(req.stream.read().decode('utf-8'))
        if('x' in text):
            droneState.x = text['x']
        if('y' in text):
            droneState.y = text['y']
        if('z' in text):
            if(text['z'] >= droneState.min_z):
                droneState.z = text['z']
            else:
                droneState.z = droneState.min_z
        droneState.cleared = False
        response = {"status":"okay"}
        resp.body = json.dumps(response)

    def on_get(self, req, resp):
        resp.body = json.dumps(val)

class collisionHandler:
    #Collision updates the drone state variables: z
    def on_post(self, req, resp):
        print "Collision"
        req_text = json.loads(req.stream.read().decode('utf-8'))
        if('z' in req_text):
            droneState.min_z = req_text['z']
            if(droneState.min_z > droneState.z):
                droneState.z = droneState.min_z
        if(bool(req_text['coll_msg'])):
            print "Got args"
            coll = {}
            coll['first'] = req_text['coll_msg']['first']
            coll['second'] = req_text['coll_msg']['second']
            coll['third'] = req_text['coll_msg']['third']
            coll['fourth'] = req_text['coll_msg']['fourth']
            droneState.coll_msg = coll
        printState()

class stateHandler():
    #Flight should read variables from the drone state to get new flight vectors
    def on_get(self, req, resp):
        text = {
            "x":droneState.x,
            "y":droneState.y,
            "z":droneState.z,
            "coll_msg":droneState.coll_msg
        }
        resp.body = json.dumps(text)
        req_text = req.stream.read().decode('utf-8')
        if('clear' in req_text):    #Was clearing buffer requested?
            print "Clearing buffer"
            lastState = droneState
            droneState.x = 0
            droneState.y = 0
            droneState.z = 0
            droneState.coll_msg = {}
            droneState.cleared = True
        printState()

    def on_post(self, req, resp):
        pass

api = falcon.API()
api.add_route('/api/vision', visionHandler())
api.add_route('/api/collision', collisionHandler())
api.add_route('/api/state', stateHandler())
api.add_route('/api/current', currentHandler())