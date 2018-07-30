import time

a = 0

def inc():
    while(True):
        global a
        a += 1      
        time.sleep(1)
        print (a)