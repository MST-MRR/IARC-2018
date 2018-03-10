import falcon
import json
import droneState_pb2 as droneState

droneState = droneState.DroneState()

class visionHandler:
    #Vision updates the drone state variables: x,y
    def on_post(self, req, resp):
        text = json.loads(req.stream.read().decode('utf-8'))
        print(text['x'])
        print(text['y'])
        print(text['z'])

    def on_get(self, req, resp):
        resp = json.dumps({"test":"test"})

class collisionHandler:
    #Collision updates the drone state variables: z
    def on_post(self, req, resp):
        pass

class flightHandler():
    #Flight should read variables from the drone state to get new flight vectors
    def on_get(self, req, resp):
        pass

api = falcon.API()
api.add_route('/api/vision', visionHandler())
api.add_route('/api/collision', collisionHandler())
api.add_route('/api/flight', flightHandler())