import matplotlib.pyplot as plt, numpy as np
from scipy.interpolate import spline

class Graph:
    def __init__(self, setpoint, title):
        self.actual_values_list = []    #List to hold the actual vehicle values
        self.time_count_list = []       #List to hold 'time' used to correlate the time and value
        self.setpoint_list = []         #List to hold setpoints
        self.timer_count = 0            #Counter used to simulate passage of time
        self.title = title
        self.setpoint = setpoint

    def update(self, update_value):
        self.actual_values_list.append(update_value)    #Appending to the actual values list
        self.timer_count += 1                           #Incrementing the timer
        self.setpoint_list.append(self.setpoint)      #Appending to the setpoint list
        self.time_count_list.append(self.timer_count)   #Appending to the time list
    
    def display(self):
        #Performing operations to plot the data onto the graph
        time_sm = np.array(self.time_count_list)
        time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
        values_smooth = spline(self.time_count_list, self.actual_values_list, time_smooth)
        plt.plot(time_smooth, values_smooth)
        plt.plot(self.time_count_list, self.setpoint_list)  #Plotting the setpoint in correlation with the time 
        plt.xlim((0, self.timer_count))                     #Plotting the x values from the time list 
        plt.ylim((min(self.actual_values_list)-0.5, max(self.actual_values_list)+0.5))  #Plotting the y values
        plt.xlabel('time')
        plt.ylabel('PID (PV)')
        plt.title('%s PID' % self.title)
        plt.grid(True)
        plt.show()