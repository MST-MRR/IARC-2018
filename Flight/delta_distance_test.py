import numpy as np
import time
from Realsense import Realsense
from ATC import Tower
from mrrdt_vision.obj_detect.roomba_cnn import RoombaDetector
from timeit import default_timer as timer
from sklearn.preprocessing import normalize
import math

class SimpleDroneAI():
    # camera fov
    CAMERA_FIELD_OF_VIEW = 55.95
    # altitude to hover at after takeoff in meters
    TAKEOFF_HEIGHT = 1.9
    # roomba speed (m/s)
    ROOMBA_SPEED = .33
    # speed to follow roombas at normally
    ROOMBA_TRACKING_SPEED = .3
    # hover if drone goes beyond this speed
    DRONE_HARD_SPEED_LIMIT = 1
#    DRONE_HARD_SPEED_LIMIT = 1.4*ROOMBA_TRACKING_SPEED
    # stop when we are this many meters away from the roomba
    ROOMBA_DISTANCE_THRESHOLD = .05
    # hover after losing the roomba for this much time in seconds
    MAX_LOST_TARGET_TIME = 1
    # roomba detector threshold
    ROOMBA_DETECTOR_THRESHOLD = .9
    # initializing roomba interaction bool
 #   ATTEMPTING_ROOMBA_INTERACTION = False
 #   ROOMBA_INTERACTION_TIME = 0
    initialized_values = False
    time_final = 0
    dist_final = 4

    def __init__(self):
        self._roomba_detector = RoombaDetector(threshold=self.ROOMBA_DETECTOR_THRESHOLD)
        self._tower = Tower(in_simulator=False)
        self._time_since_last_roomba = 0

    def is_drone_above_speed_limit(self):
        return self._tower.speed > self.DRONE_HARD_SPEED_LIMIT

    def get_meters_per_pixel(self, img):
        image_width = img.shape[1]
        image_width_in_meters = 2*math.tan(math.radians(self.CAMERA_FIELD_OF_VIEW/2.)) * self._tower.altitude
        return image_width_in_meters / image_width

    def get_velocity_vector2d(self, img, start, goal, speed):
	
        dist = float(np.sqrt(np.sum((goal-start)**2)))*self.get_meters_per_pixel(img)
        x_vel, y_vel = min(dist if dist > self.ROOMBA_DISTANCE_THRESHOLD else 0, speed)*normalize((goal-start).reshape(-1, 1), axis=1)
        if self.initialized_values == False:
		self.dist_final = (goal-start)*self.get_meters_per_pixel(img)
		self.time_final = timer()
		self.initialized_values = True
        elif self.initialized_values == True:
		time_initial = self.time_final
		dist_initial = self.dist_final
		self.dist_final = (goal-start)*self.get_meters_per_pixel(img)
		self.time_final = timer()
		
		delta_time = self.time_final - time_initial
		delta_dist = self.dist_final - dist_initial
		
		print("delta distance: ", delta_dist)
		print("delta_time: ", delta_time)
		if delta_dist[1] < 0 and delta_dist[0] > 0:
		
			roomba_direction_angle =  360 + math.degrees(math.atan(delta_dist[1]/delta_dist[0]))
		elif delta_dist[0] < 0:
			roomba_direction_angle =  180 + math.degrees(math.atan(delta_dist[1]/delta_dist[0]))
	
		else:
			roomba_direction_angle =  math.degrees(math.atan(delta_dist[1]/delta_dist[0]))
		print("Angle of Roomba w.r.t.", roomba_direction_angle)
#      if self.ATTEMPTING_ROOMBA_INTERACTION == False:
	if self._tower.altitude > self._tower.LAND_ALTITUDE:
			if dist >  self.ROOMBA_DISTANCE_THRESHOLD:
				return np.array([-y_vel, x_vel, 0])
				
			else:
				return np.array([-y_vel,x_vel,0])

			if (180 -math.degrees(math.atan(delta_dist[0]/delta_dist[1])) - self._tower.angle_from_goal()) > 70:
				print("Interacting")
				return np.array([-y_vel,x_vel,0])       

			else:
				print("Roomba going towards goal") 
				return np.array([-y_vel,x_vel,0])       

	elif self._tower.altitude < self.TAKEOFF_HEIGHT:
			return np.array([-y_vel, x_vel, .1])
#      elif self.ATTEMPTING_ROOMBA_INTERACTION == True:
#		if timer() - self.ROOMBA_INTERACTION_TIME >= 10:
#			self.ATTEMPTING_ROOMBA_INTERACTION = False
#			self._tower.pid_flight_controller.target_altitude = 1.25
#			self._tower.hover()
#			time.sleep(10)
#		else:
#			ATTEMPTING_ROOMBA_INTERACTION = True

    def follow_nearest_roomba(self, img):
        h, w = img.shape[:2]
        drone_midpoint = np.asarray([w/2, h/2])
        roombas = self._roomba_detector.detect(img)

        if self.is_drone_above_speed_limit():
        	 print("Correcting max ~~~~~~~~~~~~~~~~~~~~~~~~~~")
		
		 self._tower.hover()
        	 time.sleep(2)
        if roombas:
            roomba_midpoints = np.asarray([roomba.center for roomba in roombas])
            target_idx = np.argmin(np.sum((roomba_midpoints-drone_midpoint)**2, axis=1))
            target = roombas[target_idx]
            velocity = self.get_velocity_vector2d(img, drone_midpoint, roomba_midpoints[target_idx], self.ROOMBA_TRACKING_SPEED)
            self._tower.fly(velocity)
            self._time_since_last_roomba = timer()
        elif timer() - self._time_since_last_roomba >= self.MAX_LOST_TARGET_TIME:
           	 print("LOST ROOMBA")
		 self._tower.hover()
		 self.initialized_values = False
		
	 
    def run(self):
        with Realsense((640, 480)) as realsense:
            try:
                self._tower.connected.wait()
                print("Taking off")
                self._tower.takeoff(self.TAKEOFF_HEIGHT)
                self._tower.takeoff_completed.wait()
                print("Done taking off")
		
                while True:
                    status, color, depth = realsense.next()
                    if status:
                        self.follow_nearest_roomba(color)
			realsense.render(color)
            except KeyboardInterrupt as e:
                print('Quitting...')

if __name__ == '__main__':
    ai = SimpleDroneAI()
    ai.run()
