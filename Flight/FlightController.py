from AutonomousFlight import FlightVector
from ATC import Tower


class Controller(object):
    t = None
    vector_to_fly = None
    def __init__(self):
        self.t = Tower()
        

    def connect_to_drone(self):
        self.t.initialize()

    def disarm(self):
        self.t.disarm_drone()
    
    def takeoff(self, desired_alt, desired_angle = None):
        self.t.takeoff(desired_alt, desired_angle)
    
    def pitch(self, desired_speed):
        # TODO: Add distance to function
        # Take current flightvectr and resend it but only changing the pitch
        vector_to_fly = self.t.last_flight_vector
        vector_to_fly.x = desired_speed
        self.t.fly(vector_to_fly)
    
    def roll_right(self, desired_speed):
        vector_to_fly = self.t.last_flight_vector
        vector_to_fly.y = desired_speed
        self.t.fly(vector_to_fly)

    def hover(self, desired_alt = None):
        if desired_alt is None:
            self.t.hover(self.t.last_hover_altitude)
        else:   
            self.t.hover(desired_alt)

    def land(self, stop=False):
        self.t.land(stop)
    
    def get_altitude(self):
        return self.t.vehicle.location.global_relative_frame.alt

    def get_current_yaw(self):
        return self.t.get_yaw_deg()

    def yaw(self, desired_angle):
        # Don't call hover change value according 
        self.t.hover(self.t.last_hover_altitude, desired_angle)
        


