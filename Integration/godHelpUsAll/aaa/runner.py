import threading

from a import inc

thing = threading.Thread(target=inc)

thing.start()



def something(a):
    while True:
        print "Doing the thing"
        print a
        sleep(.5)