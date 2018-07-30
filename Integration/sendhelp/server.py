import StateSync

print "Server starting"
a = StateSync.StateSync()
a.bindSocket()

while(True):
    print "Tick"