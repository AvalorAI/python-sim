import numpy 

class Quadcopter:

    def __init__(self):
        
        self.velocity = [0,0,0]
        self.acceleration = [0,0,0]
        self.attitude = numpy.array([1,0,0,0])
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.heading = 0
        self.airspeed = 0
        self.angular_speed = [0,0,0]


