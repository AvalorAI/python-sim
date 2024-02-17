import random

class Barometer():
        
    def __init__(self, pressure):
         self.pressure = pressure
        

    def simulate_pressure_change(self):

            pressure_change = random.uniform(-.5, .5)
            self.pressure += pressure_change

            return pressure_change

    def altitude_from_barometer(self):

        self.simulate_pressure_change()

        alt = (1 - (self.pressure / 1013.25) ** 0.190284) * 145366.45 * 0.3048

        return alt * 1000
