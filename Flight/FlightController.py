from AutonomousFlight import FlightVector
from ATC import Tower


class Controller(object):
    def __init__(self):
        t = Tower()

    def connect_to_drone(self):
        self.t.initialize()
    
    def takeoff(self, desired_alt, desired_angle = None):
        self.t.takeoff(desired_alt, desired_angle)
    
    def pitch(self, desired_speed, backwards = False):
        # TODO: Add distance to function
        # Take current flightvectr and resend it but only changing the pitch
        vector_to_fly = self.t.last_flight_vector
        if (backwards):
            desired_speed *= -1

        vector_to_fly.x = desired_speed
        self.t.fly(vector_to_fly)
    
    def roll_right(self, desired_speed):
        vector_to_fly = self.t.last_flight_vector
        vector_to_fly.y = desired_speed
        self.t.fly(vector_to_fly)
    
    def roll_left(self, desired_speed):
        vector_to_fly = self.t.last_flight_vector
        vector_to_fly.y = desired_speed*(-1)
        self.t.fly(vector_to_fly)

    def hover(self):
        self.t.hover(self.t.last_hover_altitude)



