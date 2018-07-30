import zmq
import json
import time

class StateSync:
    pos_z = 0
    coll_msg = {}
    sock = {}
    context = {}

    def __init__(self):
        self.context = zmq.Context()
        self.sock = self.context.socket(zmq.REP)

    def bindSocket(self):
        self.sock.bind("tcp://127.0.0.1:5676")

    #Send a message to the API as collision
    def sendCollision(self, z, msg = {}):
        s_time = time.time()
        text = {'z':z, 'coll_msg':msg}
            
        self.sock.connect("tcp://127.0.0.1:5678")
        # r = requests.post(EndPoint.COLLISION, data = json.dumps(text))
        self.sock.send(json.dumps(text))

        if((time.time() - s_time) > (1.0/30)):
            print("Sync too slow!")
        return

    #Get drone state
    def getState(self, clearBuf = False):
        s_time = time.time()
        text = {"sender":self.sender}
        if(clearBuf):
            text['clear'] = True
        r = requests.get(EndPoint.FLIGHT, data = text)
        resp_t = json.loads(r.text)
        self.pos_z = resp_t['z']
        self.coll_msg = resp_t['coll_msg']
        
        if((time.time() - s_time) > (1.0/30)):
            print("Sync too slow!")

        return r