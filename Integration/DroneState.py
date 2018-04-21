class DroneState:
    x = 0
    y = 0
    z = 0
    min_z = 0
    coll_msg = {}
    state = "undefined"

    def __init__(self):
        pass
    
    def getState(self):
        return {
            'x':self.x,
            'y':self.y,
            'z':self.z,
            'min_z':self.min_z,
            'state':self.state,
            'coll_msg':self.coll_msg
        }
    
    def update(self, var, val):
        if(var == "x"):
            x = val
        elif(var == "y"):
            y = val
        elif(var == "z"):
            z = val
        elif(var == "min_z"):
            min_z = val
        else:
            state = val