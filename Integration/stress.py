import StateSync
import time

a = StateSync.StateSync("vision")
b = StateSync.StateSync("flight")
c = StateSync.StateSync("collision")

start_time = time.time()
i = 0
x = False

print start_time
while(time.time() - start_time < 1):
    a.sendVision(1,1,5)
    b.getState()
    c.sendCollision(10 * int(x))
    i += 1
    x = not x

print("done")
print("Time: " + str( time.time() - start_time))
print("Requests handled: " + str(i))