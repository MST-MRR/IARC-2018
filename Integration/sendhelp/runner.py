import threading

def runner():
    print "This is a runner"

print "Running"

array = []
for i in range(0,5):
    array.append(threading.Thread(target=runner))
    array[i].start()