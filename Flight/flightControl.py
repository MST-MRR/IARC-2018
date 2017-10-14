import PID, time, math, os

class PIDController:
    def __init__(self, setpoint, startVal, sampleTime, PConst, IConst, DConst):
        self.setpoint = setpoint
        self.AxisPID = PID.PID(PConst,IConst, DConst)
        self.AxisPID.setSampleTime(sampleTime)
        self.AxisPID.setpoint = setpoint
        self.currentVal = startVal
    def update(self, updateValue):
        self.AxisPID.update(updateValue)
        self.currentVal += self.AxisPID.output
        return self.currentVal
     
