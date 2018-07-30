import zmq
import json
import sys
import time
import threading

def Test():
    # ZeroMQ Context    
    context = zmq.Context()

    # Define the socket using the "Context"
    sock = context.socket(zmq.REQ)
    sock.connect("tcp://127.0.0.1:5678")
    print "Creating test"
    thing = {
        "z": 3
    }

    start_time = time.time()
    val = 0

    # Send a "message" using the socket
    while (time.time() - start_time < 1):
        sock.send(json.dumps(thing))
        sock.recv()
        val += 1
    print val