import math

RADIUS_EARTH = 6371000 # meters

class GPS():

    def __init__(self, lat, lon):
        self.home_lat = lat
        self.home_lon = lon

    def get_gps_position(self,x,y):
        
        # y = lat
        a_lat = math.sin(0.5*y/RADIUS_EARTH)**2

        if y < 0:
            sign = -1
        else:
            sign= 1

        add_lat = sign*math.degrees(2*math.sin(math.sqrt(a_lat)))*1e7

        # x = lon
        a_lon = math.sin(0.5*x/RADIUS_EARTH)**2

        lat_rad = math.radians(self.home_lat/1e7)

        if x < 0:
            sign = -1
        else:
            sign= 1
        
        add_lon = sign*math.degrees(2*math.sin(math.sqrt(a_lon))/math.cos(lat_rad))*1e7

        return self.home_lat + add_lat, self.home_lon + add_lon


        




