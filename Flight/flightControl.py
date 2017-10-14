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
    def setSetPoint(self, setpoint):
        self.AxisPID.setpoint = setpoint
    def setP(self, PConst):
        self.AxisPID.setKp(PConst)
    def setI(self, IConst):
        self.AxisPID.setKi(IConst)
    def setD(self, DConst):
        self.AxisPID.setKd(DConst)
    def setSampleTime(self, sampleTime):
        self.AxisPID.setSampleTime(sampleTime)
    def clear(self):
        self.AxisPID.clear()
    def setWindup(self, windup):
        self.AxisPID.setWindup(windup)
