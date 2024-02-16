import random
import numpy as np

class IMU:

    def __init__(self):

        self.gravity = 9.81
        self.mass = 1.0
        self.l = 0.5 # Distance between the quadcopter's center and its actuators.
        self.k = 4.905 # Thrust coefficient
        self.kd = 0.1 # Drag coefficient

        self.state = {
            'x': 0, # --> GPS lon
            'y': 0, # --> GPS lat
            'z': 0, # --> alt in mm 
            'vn': 0,
            've': 0,
            'vd': 0,
            'eph': 0,
            'epv': 0,
            'cog': 0, 
            'v_x': 0, # --> vx_cm
            'v_y': 0, # --> vy_cm
            'v_z': 0, # --> vz_cm
            'attitude_quaternion': np.array([1,0,0,0]),
            'angular_v': np.array([0, 0, 0]), # --> rollspeed, pitchspeed, yawspeed in rad/s
            'barometer': 1013.25
        }
    
    def forces_from_actuators(self, actuators):
        f_total = sum(actuators)
        torques = np.array([
            ((actuators[0] + actuators[3]) - (actuators[1] + actuators[2])) * self.l * self.k,  # Roll
            ((actuators[0] + actuators[1]) - (actuators[2] + actuators[3])) * self.l * self.k,  # Pitch
            (actuators[0] - actuators[1] + actuators[2] - actuators[3]) * self.kd          # Yaw
        ])
        return f_total, torques

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

    def dynamics(self, state, actuators):
        f, torques = self.forces_from_actuators(actuators)
        
        # Compute the world frame thrust components
        R = self.rotation_matrix(state['roll'], state['pitch'], state['yaw'])
        world_thrust = R @ np.array([0, 0, f*self.k])
        
        dx = state['v_x']
        dy = state['v_y']
        dz = state['v_z']
        
        dvx = world_thrust[0] / self.m
        dvy = world_thrust[1] / self.m
        dvz = world_thrust[2] / self.m - self.g
        
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
    
        return {
            'x': dx, 
            'y': dy, 
            'z': dz,
            'v_x': dvx, 
            'v_y': dvy, 
            'v_z': dvz,
            'roll': droll,
            'pitch': dpitch,
            'yaw': dyaw,
            'angular_v': dangular_v
        }

    def update_state_rk4(self, state, actuators, dt, ground_level):
        k1 = self.dynamics(state, actuators)
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
    
    def simulate_pressure_change(self):

        pressure_change = random.uniform(-.5, .5)
        self.state['barometer'] += pressure_change

    def altitude_from_barometer(self):

        self.simulate_pressure_change()

        alt = (1 - (self.drone['barometer'] / 1013.25) ** 0.190284) * 145366.45 * 0.3048

        self.drone['alt'] = alt * 1000

        