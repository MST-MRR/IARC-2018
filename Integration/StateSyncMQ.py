import zmq

class StateSyncMQ:
    min_z = 0
    zmq_inst = None
    sock = None

    def __init__(self):
        self.zmq_inst = zmq.Context()  
        self.sock = self.zmq_inst.socket(zmq.SUB)
        self.sock.connect("tcp://127.0.0.1")
        print("StateSyncMQ constructor created")

    def getZ(self):
        return self.sock.recv_string()

class SSMQSender:
    min_z = 0
    zmq_inst = None
    sock = None
    
    def __init__(self):
        self.zmq_inst = zmq.Context()
        self.sock = self.zmq_inst.socket(zmq.PUB)
        self.sock.bind("tcp://127.0.0.1:5001")
    
    def sendZ(self, Z):
        self.sock.send_string(str(Z))