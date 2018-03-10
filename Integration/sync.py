import droneState_pb2 as droneState
import requests
import time

d = droneState.DroneState()
r = requests.get('https://get.gazebosim.org/')