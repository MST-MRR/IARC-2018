import matplotlib.pyplot as plt, numpy as np
from scipy.interpolate import spline

class Graph:
    def __init__(self, setpoint, title):
        self.actual_values_list = []
        self.time_count_list = []
        self.setpoint_list = []
        self.timer_count = 0
        self.title = title
        self.setpoint = setpoint

    def update(self, update_value):
        self.actual_values_list.append(update_value)
        self.timer_count += 1
        self.setpoint_list.append(self.setpoint)
        self.time_count_list.append(self.timer_count)
    
    def display(self):
        time_sm = np.array(self.time_count_list)
        time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
        values_smooth = spline(self.time_count_list, self.actual_values_list, time_smooth)
        plt.plot(time_smooth, values_smooth)
        plt.plot(self.time_count_list, self.setpoint_list)
        plt.xlim((0, self.timer_count))
        plt.ylim((min(self.actual_values_list)-0.5, max(self.actual_values_list)+0.5))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('%s PID' % self.title)
        plt.grid(True)
        plt.show()