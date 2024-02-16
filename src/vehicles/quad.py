import numpy 

class quadcopter:

    def __init__(self):
        
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.attitude = numpy.array([1,0,0,0])
        self.lat = 0
        self.lon = 0
        self.alt = 0
        

