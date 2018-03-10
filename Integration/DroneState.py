class DroneState:
    x = 0
    y = 0
    z = 0
    state = "undefined"

    def init(self):
        pass
    
    def getState(self):
        return {
            'x':self.x,
            'y':self.y,
            'z':self.z,
            'state':self.state
        }
    
    def update(self, var, val):
        if(var == "x"):
            x = val
        elif(var == "y"):
            y = val
        elif(var == "z"):
            z = val
        else:
            state = val