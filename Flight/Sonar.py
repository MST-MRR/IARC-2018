class Sonar:
    
    #default constructor
    #trigger, echo, and side are passes through and set accordingly
    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo
        #set GPIO direction (IN / OUT)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    #performs operations to get the approximate distance
    #returns distance to calling function
    def get_distance(self):
        GPIO.output(self.trigger, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.1)
        GPIO.output(self.trigger, False)

        self.startTime = time.time()
        self.stopTime = time.time()

        # save StartTime
        while GPIO.input(self.echo) == 0:
            self.startTime = time.time()

        # save time of arrival
        while GPIO.input(self.echo) == 1:
            self.stopTime = time.time()

        # time difference between start and arrival
        self.timePassed = self.stopTime - self.startTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distance = (self.timePassed * 34300) / 2

        return self.distance
    