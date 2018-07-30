from ATC import Tower
import time
import math

T = Tower(in_simulator=False)
T.connected.wait()


while True:
	print(T.altitude)
	print("Degrees from North: ",(T.angle_from_goal()))
	print("goal angle: ", T.goal_angle)
	print("yaw angle: ", T.drone_yaw_angle())	
	
