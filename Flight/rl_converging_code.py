import numpy as np

from Realsense import Realsense
from ATC import Tower
from mrrdt_vision.obj_detect.roomba_cnn import RoombaDetector
from timeit import default_timer as timer
from sklearn.preprocessing import normalize
import math

class SimpleDroneAI():
    # camera fov
    CAMERA_FIELD_OF_VIEW = 55.95
    ROOMBA_TRACKING_SPEED = .4     # altitude to hover at after takeoff in meters
    TAKEOFF_HEIGHT = 1.25
    # roomba speed (m/s)
    ROOMBA_SPEED = .33
    # hover if drone goes beyond this speed
    DRONE_HARD_SPEED_LIMIT = 1.2*ROOMBA_TRACKING_SPEED
    # stop when we are this many meters away from the roomba
    ROOMBA_DISTANCE_THRESHOLD = .2
    # hover after losing the roomba for this much time in seconds
    MAX_LOST_TARGET_TIME = 10
    # roomba detector threshold
    ROOMBA_DETECTOR_THRESHOLD = .9
    initialized_velocity_data = False
    dist_final = 10.0
    time_final = 0.0
    slow_down_distance = 1.0
    converging_speed_threshold = 5
    follow_time = 5.0
    time_over_roomba = 0
    previous_velocity = np.array([])
    roomba_interaction = False
    roomba_interaction_time_start = 0
    interaction_time = 5   
    v_max_dif = 0.1

    def __init__(self):
        self._roomba_detector = RoombaDetector(threshold=self.ROOMBA_DETECTOR_THRESHOLD)
        self._tower = Tower(in_simulator=False)
        self._time_since_last_roomba = 0

    def is_drone_above_speed_limit(self):
        return self._tower.vehicle.airspeed > self.DRONE_HARD_SPEED_LIMIT

    def get_meters_per_pixel(self, img):
        image_width = img.shape[1]
        image_width_in_meters = 2*math.tan(math.radians(self.CAMERA_FIELD_OF_VIEW/2.)) * self._tower.altitude
        return image_width_in_meters / image_width

    def get_velocity_vector2d(self, img, start, goal, speed):
#      if self.ATTEMPTING_ROOMBA_INTERACTION == False:
	print("ROOMBA visible")
        if self.initialized_velocity_data == False:
            self.time_final = timer()
            self.dist_final = (goal-start)*self.get_meters_per_pixel(img)
            self.initialized_velocity_data = True
	    v_drone = np.array([0.0,0.0,0.0])
        else:
            time_initial = self.time_final
            dist_initial = self.dist_final
            self.dist_final = (goal-start)*self.get_meters_per_pixel(img)
            self.time_final = timer()
            delta_dist_wrt_drone = self.dist_final - dist_initial
            delta_time = self.time_final - time_initial
            current_drone_velocity = np.array([self._tower.vehicle.velocity[1],self._tower.vehicle.velocity[0]])
            delta_dist_wrt_goal = delta_dist_wrt_drone - delta_time*current_drone_velocity
            v_roomba_wrt_drone = delta_dist_wrt_drone/delta_time
            v_drone=np.array([0.0,0.0,0.0])
            i = 0
            
            for dimensions in self.dist_final:
                if dimensions < (start[i])*self.get_meters_per_pixel(img):
                    sign_multiplier = -1.0
                else:
                    sign_multiplier = 1.0
#                if np.abs(current_drone_velocity[i] - p velocity) > .1
#			v_drone[i] = previous_    
                if abs(dimensions) <= self.slow_down_distance:
#                    v_drone[i] = self.v_max_dif/self.slow_down_distance*dimensions + v_roomba_wrt_drone[i] + current_drone_velocity[i]
               	     v_drone[i] = self.v_max_dif/self.slow_down_distance*dimensions
                else:
#                    v_drone[i] = sign_multiplier*self.v_max_dif + v_roomba_wrt_drone[i] + current_drone_velocity[i]
                     v_drone[i] = sign_multiplier*self.v_max_dif
                if abs(v_drone[i]) > abs(self.ROOMBA_SPEED + self.v_max_dif):
                    v_drone[i] = self.v_max_dif*sign_multiplier
                i = i + 1
            self.previous_velocity = np.array(v_drone)
            drone_speed = abs(v_drone/np.sqrt(np.sum(v_drone**2)))
            roomba_speed_wrt_drone = abs(np.sqrt(np.sum(v_roomba_wrt_drone**2)))
            absolute_distance = abs(np.sqrt(np.sum(self.dist_final**2)))
#           print("delta_dist: ", delta_dist_wrt_drone)
	    print("dist_final: ", self.dist_final[0], self.dist_final[1])   

#            if roomba_speed_wrt_drone < self.converging_speed_threshold and absolute_distance < self.ROOMBA_DISTANCE_THRESHOLD:
            if absolute_distance < self.ROOMBA_DISTANCE_THRESHOLD:
                if (timer() - self.time_over_roomba) >= self.follow_time and self.roomba_interaction == False:
			self._tower.pid_flight_controller.target_altitude = .4
			self.roomba_interaction_time_start = timer()
			self.roomba_interaction = True
			print("Starting Interaction")
			print("$"*800)
	        elif (timer() - self.time_over_roomba) >= self.follow_time and self.roomba_interaction == True:
			if (timer() - self.roomba_interaction_time_start) > self.interaction_time:
				self.roomba_interaction = False
				self._tower.pid_flight_controller.target_altitude = self.TAKEOFF_HEIGHT
			        self.roomba_interaction_time_start = 0
				self.time_over_roomba = timer()
				print("Ending Interaction")
				print("%"*80)
			else:
				print("Continuing Interaction")
                else:
                	self.time_over_roomba = timer()
	print("Desired velocity: ", v_drone[0],"\t|\t", v_drone[1])
        print("~"*300)
        return np.array([v_drone[0],v_drone[1],0])

    def follow_nearest_roomba(self, img):
        h, w = img.shape[:2]
        drone_midpoint = np.asarray([w/2, h/2])
        roombas = self._roomba_detector.detect(img)

        if self.is_drone_above_speed_limit():
            self._tower.hover()
        
        if roombas:
            roomba_midpoints = np.asarray([roomba.center for roomba in roombas])
            target_idx = np.argmin(np.sum((roomba_midpoints-drone_midpoint)**2, axis=1))
            target = roombas[target_idx]
            velocity = self.get_velocity_vector2d(img, drone_midpoint, roomba_midpoints[target_idx], self.ROOMBA_TRACKING_SPEED)
            self._tower.fly(velocity)
            self._time_since_last_roomba = timer()
        elif timer() - self._time_since_last_roomba >= self.MAX_LOST_TARGET_TIME:
            self._tower.hover()
	    print("No Roomba In Frame")
	    self.initialized_velocity_data = False

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
            except KeyboardInterrupt as e:
                print('Quitting...')

if __name__ == '__main__':
    ai = SimpleDroneAI()
    ai.run()
	
