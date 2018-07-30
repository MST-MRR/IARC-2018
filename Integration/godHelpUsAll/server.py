import zmq
import json
import threading

def Server():
    # ZeroMQ Context
    context = zmq.Context()

    # Define the socket using the "Context"
    sock = context.socket(zmq.REP)
    sock.bind("tcp://127.0.0.1:5678")

    print "Server created"
    print "Server run"
    # Run a simple "Echo" server
    while True:
        message = sock.recv()
        sock.send(message)
        # print (message)
        print json.loads(message)