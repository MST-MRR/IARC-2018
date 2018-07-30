import StateSync
import time

a = StateSync.StateSync()
a.bindSocket()
c = StateSync.StateSync()
print "aaa"

start_time = time.time()
i = 0
x = False

print start_time
while(time.time() - start_time < 1):
    c.sendCollision(10 * int(x))
    i += 1
    x = not x

print("done")
print("Time: " + str( time.time() - start_time))
print i