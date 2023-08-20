import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

# Constants
g = 9.81
m = 1.0
l = 0.5
k = 4.905
kd = 0.1

# Initialize State and History for Visualization
state = {
    'x': 0,
    'y': 0,
    'z': 0,
    'v_x': 0,
    'v_y': 0,
    'v_z': 0,
    'roll': 0,
    'pitch': 0,
    'yaw': 0,
    'angular_v': np.array([0, 0, 0])
}

history = {key: [] for key in state.keys()}
actuator_history = []
force_history = []

def forces_from_actuators(actuators):
    f_total = sum(actuators)
    torques = np.array([
        ((actuators[0] + actuators[3]) - (actuators[1] + actuators[2])) * l * k,  # Roll
        ((actuators[0] + actuators[1]) - (actuators[2] + actuators[3])) * l * k,  # Pitch
        (actuators[0] - actuators[1] + actuators[2] - actuators[3]) * kd          # Yaw
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

def dynamics(state, actuators):
    f, torques = forces_from_actuators(actuators)
    
    # Compute the world frame thrust components
    R = rotation_matrix(state['roll'], state['pitch'], state['yaw'])
    world_thrust = R @ np.array([0, 0, f*k])
    
    dx = state['v_x']
    dy = state['v_y']
    dz = state['v_z']
    
    dvx = world_thrust[0] / m
    dvy = world_thrust[1] / m
    dvz = world_thrust[2] / m - g
    
    # Moment of inertia (for simplicity, assuming it's the same for all axes)
    I_xx = I_yy = I_zz = 0.02  # This is just a placeholder, adjust based on your quadcopter's properties

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

def update_state_rk4(state, actuators, dt):
    k1 = dynamics(state, actuators)
    k2 = dynamics({key: state[key] + 0.5 * dt * k1[key] for key in state.keys()}, actuators)
    k3 = dynamics({key: state[key] + 0.5 * dt * k2[key] for key in state.keys()}, actuators)
    k4 = dynamics({key: state[key] + dt * k3[key] for key in state.keys()}, actuators)
    
    new_state = {}
    for key in state.keys():
        new_state[key] = state[key] + dt * (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) / 6
    return new_state


def save_flight_data_to_csv(history, filename="flight_data.csv"):
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = list(history.keys())  # gets all the keys (e.g., 'x', 'y', 'z', ...)
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for i in range(len(history['x'])):
            row_data = {key: history[key][i] for key in history}
            writer.writerow(row_data)

# Constants
dt = 0.01
duration_per_phase = 200  # For example, each phase lasts for 10 seconds.

# Actuator configurations FR, FL, RF, RR
take_off = [0.6, 0.6, 0.6, 0.6]
hover = [0.5, 0.5, 0.5, 0.5]
forward_flight = [0.5, 0.5, 0.51, 0.51]
roll = [0.5, 0.6, 0.5, 0.5]
brake =  [0.591, 0.59, 0.589, 0.59]
heading_change = [0.48, 0.5, 0.52, 0.5]  
descent = [0.4, 0.4, 0.4, 0.4]

phases = ['take_off', 'descent', 'hover', 'forward_flight']
actuator_configs = {
    'take_off': take_off,
    'hover': hover,
    'forward_flight': forward_flight,
    'heading_change': heading_change,
    'descent': descent,
    'brake' : brake,
    'roll': roll
}

# Create an initial 3D plot for visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], lw=2)
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([0, 10])

def update_3D_trajectory(x, y, z):
    line.set_data(x, y)
    line.set_3d_properties(z)
    plt.draw()
    plt.pause(0.01)

# Simulation loop
for phase in phases:
    actuators = actuator_configs[phase]
    for _ in range(duration_per_phase):
        actuator_history.append(actuators)
        f, torques = forces_from_actuators(actuators)
        force_history.append((f, torques))
        state = update_state_rk4(state, actuators, dt)
        for key in state.keys():
            history[key].append(state[key])
        
        # Update the live 3D plot with the new position
        update_3D_trajectory(history['x'], history['y'], history['z'])

plt.show()


# Compute acceleration (difference in velocity over dt)
acceleration_z = np.diff(history['v_z']) / dt
# Pad with 0 at the beginning to make it the same length as history
acceleration_z = np.insert(acceleration_z, 0, 0)

# Compute angular acceleration (difference in angular velocity over dt for each axis)
angular_acceleration = (np.diff(np.array(history['angular_v']), axis=0) / dt).tolist()
# Pad with 0s at the beginning to make it the same length as history
angular_acceleration = [[0,0,0]] + angular_acceleration

plt.figure(figsize=(15, 35))

# Position

# # X distance
# plt.subplot(7, 2, 1)  # Adjusted row count to 6
# plt.plot(history['x'])
# plt.title("X Distance over Time")
# plt.ylabel("X Distance (m)")

# # Y distance
# plt.subplot(7, 2, 2)  # Adjusted row count to 6
# plt.plot(history['y'])
# plt.title("Y Distance over Time")
# plt.ylabel("Y Distance (m)")

# Z distance
plt.subplot(7, 2, 3)
plt.plot(history['z'])
plt.title("Altitude over Time")
plt.ylabel("Altitude (m)")

# Velocity

# v_x
plt.subplot(7, 2, 4)
plt.plot(history['v_x'])
plt.title("Vx over Time")
plt.ylabel("Vx (m/s)")

# v_y
plt.subplot(7, 2, 5)
plt.plot(history['v_y'])
plt.title("Vy over Time")
plt.ylabel("Vy (m/s)")

# v_z 
plt.subplot(7, 2, 6)
plt.plot(history['v_z'])
plt.title("Vz over Time")
plt.ylabel("Vz (m/s)")


# Acceleration

plt.subplot(7, 2, 13)
plt.plot(acceleration_z)
plt.title("Acceleration in Z over Time")
plt.ylabel("Acceleration (m/s^2)")

# Attitude

# Roll
plt.subplot(7, 2, 7)
plt.plot(history['roll'])
plt.title("Roll Angle over Time")
plt.ylabel("Roll (radians)")

# Pitch
plt.subplot(7, 2, 8)
plt.plot(history['pitch'])
plt.title("Pitch Angle over Time")
plt.ylabel("Pitch (radians)")

# Yaw
plt.subplot(7, 2, 9)
plt.plot(history['yaw'])
plt.title("Yaw Angle over Time")
plt.ylabel("Yaw (radians)")

# Angular Velocity
plt.subplot(7, 2, 10)
labels = ['X', 'Y', 'Z']
for i in range(3):
    plt.plot([v[i] for v in history['angular_v']], label=f'Angular Vel. {labels[i]}')
plt.title("Angular Velocities over Time")
plt.ylabel("Ang. Vel. (rad/s)")
plt.legend()

# Actuator Outputs and Forces
plt.subplot(7, 2, 11)
for i in range(4):
    plt.plot([act[i] for act in actuator_history], label=f'Actuator {i+1}')
plt.title("Actuator Outputs over Time")
plt.ylabel("Actuator Output")
plt.legend()

# Torque
plt.subplot(7, 2, 12)
torque_labels = ['Roll', 'Pitch', 'Yaw']
for i in range(3):
    plt.plot([t[1][i] for t in force_history], label=f'Torque {torque_labels[i]}')
plt.title("Torques over Time")
plt.ylabel("Torque")
plt.legend()

plt.tight_layout()
plt.show()

save_flight_data_to_csv(history)