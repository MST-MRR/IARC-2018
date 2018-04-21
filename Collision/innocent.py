import StateSync
#import DroneState

f = StateSync.StateSync("flight")
c = StateSync.StateSync("collision")

text = {'first':1, 'second':2, 'third':3, 'fourth':4}
c.sendCollision(5)

f.getState(True)