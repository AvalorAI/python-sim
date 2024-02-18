import numpy as np

class IMU:

    def __init__(self):

        self.gravity = 9.81
        self.mass = 1.0
        self.l = 0.5 # Distance between the quadcopter's center and its actuators.
        self.k = 4.905 # Thrust coefficient
        self.kd = 0.1 # Drag coefficient
        self.standard_pressure = 1013.25 # Barometric pressure at home pos
        self.actuators = [0,0,0,0]
        self.state = {
            'x': 0, # --> GPS lon
            'y': 0, # --> GPS lat
            'z': 0, # --> cm 
            'v_x': 0, # --> cm/s
            'v_y': 0, # --> cm/s
            'v_z': 0, # --> cm/s
            'acc_x': 0, # --> cm/s/s
            'acc_y': 0, # --> cm/s/s
            'acc_z': 0, # --> cm/s/s
            'attitude_quaternion': np.array([1,0,0,0]),
            'angular_v': np.array([0, 0, 0]), # --> rad/s            
        }
    
    def forces_from_actuators(self, actuators):
        f_total = sum(actuators)
        torques = np.array([
            ((actuators[0] + actuators[3]) - (actuators[1] + actuators[2])) * self.l * self.k,  # Roll
            ((actuators[0] + actuators[1]) - (actuators[2] + actuators[3])) * self.l * self.k,  # Pitch
            (actuators[0] - actuators[1] + actuators[2] - actuators[3]) * self.kd          # Yaw
        ])
        return f_total, torques

    
    @staticmethod
    def rotation_matrix(roll, pitch, yaw):
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        R = np.array([
            [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        
        return R

    def dynamics(self, state, actuators,dt):
        f, torques = self.forces_from_actuators(actuators)
        
        # Compute the world frame thrust components
        R = self.rotation_matrix(state['roll'], state['pitch'], state['yaw'])
        world_thrust = R @ np.array([0, 0, f*self.k])
        
        dvx = world_thrust[0] / self.mass
        dvy = world_thrust[1] / self.mass
        dvz = world_thrust[2] / self.mass - self.gravity
        
        # Moment of inertia (for simplicity, assuming it's the same for all axes)
        I_xx = I_yy = I_zz = 0.02  # This is just a placeholder, adjust based on quadcopter's properties

        # Angular accelerations
        alpha_x = torques[0] / I_xx
        alpha_y = torques[1] / I_yy
        alpha_z = torques[2] / I_zz

        # Rate of change of roll, pitch, and yaw
        droll = state['angular_v'][0]
        dpitch = state['angular_v'][1]
        dyaw = state['angular_v'][2]

        # Updating angular velocities
        dangular_v = np.array([alpha_x, alpha_y, alpha_z])

        # Compute new positions based on current velocities and time step
        x = state['x'] + state['v_x'] * dt
        y = state['y'] + state['v_y'] * dt
        z = state['z'] + state['v_z'] * dt
    
        return {
            'x': x, 
            'y': y, 
            'z': z,
            'v_x': state['v_x'] + dvx * dt,
            'v_y': state['v_y'] + dvy * dt,
            'v_z': state['v_z'] + dvz * dt,
            'acc_x': dvx,
            'acc_y': dvy,
            'acc_z': dvz,
            'roll': droll,
            'pitch': dpitch,
            'yaw': dyaw,
            'angular_v': dangular_v
        }
    
    def get_heading(self):
        # Get the yaw value from the state dictionary
        yaw_radians = self.state['yaw']

        # Convert yaw from radians to degrees
        yaw_degrees = np.degrees(yaw_radians)

        return yaw_degrees

    def get_airspeed(self):
        # Retrieve the velocity components from the state
        v_x = self.state['v_x']
        v_y = self.state['v_y']
        v_z = self.state['v_z']

        # Calculate the airspeed (magnitude of the velocity vector)
        airspeed = np.sqrt(v_x**2 + v_y**2 + v_z**2)

        return airspeed
    
    def pressure_from_altitude(self):
        # Convert altitude from cm to feet
        altitude_ft = self.state['z'] / 3.048
        # Calculate pressure using the barometric formula
        pressure = self.standard_pressure * (1 - 0.0065 * altitude_ft / 288.15) ** 5.255
        return pressure

    def update_state_rk4(self, state, actuators, dt):
        k1 = self.dynamics(state, actuators, dt)
        k2 = self.dynamics({key: state[key] + 0.5 * dt * k1[key] for key in state.keys()}, actuators)
        k3 = self.dynamics({key: state[key] + 0.5 * dt * k2[key] for key in state.keys()}, actuators)
        k4 = self.dynamics({key: state[key] + dt * k3[key] for key in state.keys()}, actuators)
        
        new_state = {}
        for key in state.keys():
            new_state[key] = state[key] + dt * (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) / 6

        # Check for ground collision
        if new_state['z'] < self.GROUND_LEVEL:
            new_state['z'] = self.GROUND_LEVEL
            new_state['v_z'] = 0  # you can also set this to some fraction of current v_z if you want it to bounce

        return new_state

        