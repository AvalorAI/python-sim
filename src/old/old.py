import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import time
import logging
from pymavlink import mavutil
from random import random


# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def crandom():
    return random()-0.5

def setup_mavlink_connection():

    global vehicle
    global n
        
    print("Waiting to connect...")
    vehicle = mavutil.mavlink_connection('tcpin:172.19.176.1:4560')
    
    msg = vehicle.recv_match(blocking = True)
    if msg.get_type() != "COMMAND_LONG":
        raise Exception("error")
    n += 1
    print(n, "<== IN: ", msg)
    
    msg = vehicle.recv_match(blocking = True)
    if msg.get_type() != "HEARTBEAT":
        raise Exception("error")
    n += 1
    print(n, "<== IN", msg)

vehicle = None
drone = {}
n = 0

setup_mavlink_connection()

time_absolute_seconds = time.time()
time_absolute_microseconds = round(time_absolute_seconds * 1e6)
time_boot_microseconds = round(time_absolute_microseconds - 30e6)
dt = 0.0001

# Constants
g = 9.81
m = 1.0
l = 0.5
k = 4.905
kd = 0.1
GROUND_LEVEL = 0
SIMULATION_TIME = 360
dt = 0.0001

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

def update_state_rk4(state, actuators, dt):
    k1 = dynamics(state, actuators)
    k2 = dynamics({key: state[key] + 0.5 * dt * k1[key] for key in state.keys()}, actuators)
    k3 = dynamics({key: state[key] + 0.5 * dt * k2[key] for key in state.keys()}, actuators)
    k4 = dynamics({key: state[key] + dt * k3[key] for key in state.keys()}, actuators)
    
    new_state = {}
    for key in state.keys():
        new_state[key] = state[key] + dt * (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) / 6

    # Check for ground collision
    if new_state['z'] < GROUND_LEVEL:
        new_state['z'] = GROUND_LEVEL
        new_state['v_z'] = 0  # you can also set this to some fraction of current v_z if you want it to bounce

    return new_state


def save_flight_data_to_csv(history, filename="flight_data.csv"):
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = list(history.keys())  # gets all the keys (e.g., 'x', 'y', 'z', ...)
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for i in range(len(history['x'])):
            row_data = {key: history[key][i] for key in history}
            writer.writerow(row_data)


def send_heartbeat():
     global n

     if t__seconds % 1000000 == 0:
        n += 1
        
        the_type        = 0     # Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type. (type:uint8_t, values:MAV_TYPE)
        autopilot       = 0     # Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
        base_mode       = 0     # System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
        custom_mode     = 0     # A bitfield for use for autopilot-specific flags (type:uint32_t)
        system_status   = 0     # System status flag. (type:uint8_t, values:MAV_STATE)
        mavlink_version = 3     # MAVLink version, not writable by user, gets added by protocol because of magic data type          , # uint8_t_mavlink_version (type:uint8_t)
        
        if vehicle != None:
            vehicle.mav.heartbeat_send(
                type                = the_type          , 
                autopilot           = autopilot         , 
                base_mode           = base_mode         , 
                custom_mode         = custom_mode       , 
                system_status       = system_status     , 
                mavlink_version     = mavlink_version   , 
            )
        
        print (n, "OUT: --> HEARTBEAT")


def send_gps():
    global n
    
    if t__microseconds % 52000 == 0:
        n += 1
        
        drone['i_lat__degE7']               = round((   47.397742   +crandom()*5e-7     )*1e7)
        drone['i_lon__degE7']               = round((   8.545594    +crandom()*5e-7     )*1e7)
        drone['i_alt__mm']                  = round((   488         +crandom()*0.05     )*1000)
        drone['i_eph__cm']                  = round((   0.3         + random()*0.001    )*100)
        drone['i_epv__cm']                  = round((   0.4         + random()*0.001    )*100)
        drone['i_vel__cm/s']                = round((   0           + random()*0.001    )*100)
        drone['i_vn__cm/s']                 = round((   0           + random()*0.001    )*100)
        drone['i_ve__cm/s']                 = round((   0           + random()*0.001    )*100)
        drone['i_vd__cm/s']                 = round((   0           + random()*0.001    )*100)
        drone['i_cog__cdeg']                = round((   0           +crandom()*0.001    )*100)


        time_usec           = time_absolute_microseconds    # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
        fix_type            = 3                             # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. (type:uint8_t)
        lat                 = drone['i_lat__degE7']         # Latitude (WGS84) [degE7] (type:int32_t)
        lon                 = drone['i_lon__degE7']         # Longitude (WGS84) [degE7] (type:int32_t)
        alt                 = drone['i_alt__mm']            # Altitude (MSL). Positive for up. [mm] (type:int32_t)
        eph                 = drone['i_eph__cm']            # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
        epv                 = drone['i_epv__cm']            # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
        vel                 = drone['i_vel__cm/s']          # GPS ground speed. If unknown, set to: 65535 [cm/s] (type:uint16_t)
        vn                  = drone['i_vn__cm/s']           # GPS velocity in north direction in earth-fixed NED frame [cm/s] (type:int16_t)
        ve                  = drone['i_ve__cm/s']           # GPS velocity in east direction in earth-fixed NED frame [cm/s] (type:int16_t)
        vd                  = drone['i_vd__cm/s']           # GPS velocity in down direction in earth-fixed NED frame [cm/s] (type:int16_t)
        cog                 = drone['i_cog__cdeg']          # Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
        satellites_visible  = 10                            # Number of satellites visible. If unknown, set to 255 (type:uint8_t)
        the_id              = 0                             # GPS ID (zero indexed). Used for multiple GPS inputs (type:uint8_t)
        yaw                 = 90                             # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north [cdeg] (type:uint16_t)
        
        if vehicle != None:
            vehicle.mav.hil_gps_send(
                time_usec           = time_usec             ,
                fix_type            = fix_type              ,
                lat                 = lat                   ,
                lon                 = lon                   ,
                alt                 = alt                   ,
                eph                 = eph                   ,
                epv                 = epv                   ,
                vel                 = vel                   ,
                vn                  = vn                    ,
                ve                  = ve                    ,
                vd                  = vd                    ,
                cog                 = cog                   ,
                satellites_visible  = satellites_visible    ,
                id                  = the_id                ,
                yaw                 = yaw                   ,
            )

    print (n, "OUT: --> GPS")


def send_state_quaternion():
    global n

    if t__microseconds % 8000 == 0:
        n += 1

        drone['f_attitude_quaternion__1']   = [
                                            float((   1           +crandom()*0        )*1),
                                            float((   0           +crandom()*0        )*1),
                                            float((   0           +crandom()*0        )*1),
                                            float((   0           +crandom()*0        )*1),
                                            ]
        drone['f_rollspeed__rad/s']         = float((   0           +crandom()*0        )*1)
        drone['f_pitchspeed__rad/s']        = float((   0           +crandom()*0        )*1)
        drone['f_yawspeed__rad/s']          = float((   0           +crandom()*0        )*1)
        drone['i_vx__cm/s']                 = round((   0           +crandom()*0.001    )*100)
        drone['i_vy__cm/s']                 = round((   0           +crandom()*0.001    )*100)
        drone['i_vz__cm/s']                 = round((   0           +crandom()*0.001    )*100)
        drone['i_ind_airspeed__cm/s']       = round((   0           + random()*0.001    )*100)
        drone['i_true_airspeed__cm/s']      = round((   0           + random()*0.3      )*100)
        drone['i_xacc__mG']                 = round((   0           +crandom()*0.001    )*100)
        drone['i_yacc__mG']                 = round((   0           +crandom()*0.001    )*100)
        drone['i_zacc__mG']                 = round((   0           +crandom()*0.001    )*100)
        
        time_usec           = time_absolute_microseconds        # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
        attitude_quaternion = drone['f_attitude_quaternion__1'] # Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation) (type:float)
        rollspeed           = drone['f_rollspeed__rad/s']       # Body frame roll / phi angular speed [rad/s] (type:float)
        pitchspeed          = drone['f_pitchspeed__rad/s']      # Body frame pitch / theta angular speed [rad/s] (type:float)
        yawspeed            = drone['f_yawspeed__rad/s']        # Body frame yaw / psi angular speed [rad/s] (type:float)
        lat                 = drone['i_lat__degE7']             # Latitude [degE7] (type:int32_t)
        lon                 = drone['i_lon__degE7']             # Longitude [degE7] (type:int32_t)
        alt                 = drone['i_alt__mm']                # Altitude [mm] (type:int32_t)
        vx                  = state['v_x']                      # Ground X Speed (Latitude) [cm/s] (type:int16_t)
        vy                  = state['v_y']                      # Ground Y Speed (Longitude) [cm/s] (type:int16_t)
        vz                  = state['v_z']                      # Ground Z Speed (Altitude) [cm/s] (type:int16_t)
        ind_airspeed        = drone['i_ind_airspeed__cm/s']     # Indicated airspeed [cm/s] (type:uint16_t)
        true_airspeed       = drone['i_true_airspeed__cm/s']    # True airspeed [cm/s] (type:uint16_t)
        xacc                = drone['i_xacc__mG']               # X acceleration [mG] (type:int16_t)
        yacc                = drone['i_yacc__mG']               # Y acceleration [mG] (type:int16_t)
        zacc                = drone['i_zacc__mG']               # Z acceleration [mG] (type:int16_t)
        
        if vehicle != None:
            vehicle.mav.hil_state_quaternion_send(
                time_usec           = time_usec             ,
                attitude_quaternion = attitude_quaternion   ,
                rollspeed           = rollspeed             ,
                pitchspeed          = pitchspeed            ,
                yawspeed            = yawspeed              ,
                lat                 = lat                   ,
                lon                 = lon                   ,
                alt                 = alt                   ,
                vx                  = vx                    ,
                vy                  = vy                    ,
                vz                  = vz                    ,
                ind_airspeed        = ind_airspeed          ,
                true_airspeed       = true_airspeed         ,
                xacc                = xacc                  ,
                yacc                = yacc                  ,
                zacc                = zacc                  ,
            )
        
        print (n, "--> HIL_STATE_QUATERNION")


def send_sensor():
    global n

    if t__microseconds % 4000 == 0:
        n += 1

        drone['f_xacc__m/s2']               = float((   0           +crandom()*0.2      )*1)
        drone['f_yacc__m/s2']               = float((   0           +crandom()*0.2      )*1)
        drone['f_zacc__m/s2']               = float((   -9.81       +crandom()*0.2      )*1)
        drone['f_xgyro__rad/s']             = float((   0           +crandom()*0.04     )*1)
        drone['f_ygyro__rad/s']             = float((   0           +crandom()*0.04     )*1)
        drone['f_zgyro__rad/s']             = float((   0           +crandom()*0.04     )*1)
        drone['f_xmag__gauss']              = float((   0.215       +crandom()*0.02     )*1)
        drone['f_ymag__gauss']              = float((   0.01        +crandom()*0.02     )*1)
        drone['f_zmag__gauss']              = float((   0.43        +crandom()*0.02     )*1)
        drone['f_abs_pressure__hPa']        = float((   95598       +crandom()*4        )*0.01)
        drone['f_diff_pressure__hPa']       = float((   0           +crandom()*0        )*0.01)
        drone['f_pressure_alt__?']          = float((   488         +crandom()*0.5      )*1)
        drone['f_temperature__degC']        = float((   0           +crandom()*0        )*1)
        
        time_usec           = time_absolute_microseconds    # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
        xacc                = drone['f_xacc__m/s2']         # X acceleration [m/s/s] (type:float)
        yacc                = drone['f_yacc__m/s2']         # Y acceleration [m/s/s] (type:float)
        zacc                = drone['f_zacc__m/s2']         # Z acceleration [m/s/s] (type:float)
        xgyro               = drone['f_xgyro__rad/s']       # Angular speed around X axis in body frame [rad/s] (type:float)
        ygyro               = drone['f_ygyro__rad/s']       # Angular speed around Y axis in body frame [rad/s] (type:float)
        zgyro               = drone['f_zgyro__rad/s']       # Angular speed around Z axis in body frame [rad/s] (type:float)
        xmag                = drone['f_xmag__gauss']        # X Magnetic field [gauss] (type:float)
        ymag                = drone['f_ymag__gauss']        # Y Magnetic field [gauss] (type:float)
        zmag                = drone['f_zmag__gauss']        # Z Magnetic field [gauss] (type:float)
        abs_pressure        = drone['f_abs_pressure__hPa']  # Absolute pressure [hPa] (type:float)
        diff_pressure       = drone['f_diff_pressure__hPa'] # Differential pressure (airspeed) [hPa] (type:float)
        pressure_alt        = drone['f_pressure_alt__?']    # Altitude calculated from pressure (type:float)
        temperature         = drone['f_temperature__degC']  # Temperature [degC] (type:float)
        fields_updated      = 7167                          # Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)
        the_id              = 0                             # Sensor ID (zero indexed). Used for multiple sensor inputs (type:uint8_t)
        
        if vehicle != None:
            vehicle.mav.hil_sensor_send(
                time_usec           = time_usec         ,
                xacc                = xacc              ,
                yacc                = yacc              ,
                zacc                = zacc              ,
                xgyro               = xgyro             ,
                ygyro               = ygyro             ,
                zgyro               = zgyro             ,
                xmag                = xmag              ,
                ymag                = ymag              ,
                zmag                = zmag              ,
                abs_pressure        = abs_pressure      ,
                diff_pressure       = diff_pressure     ,
                pressure_alt        = pressure_alt      ,
                temperature         = temperature       ,
                fields_updated      = fields_updated    ,
                id                  = the_id            ,
            )
        
        print (n, "--> HIL_SENSOR ")


def receive_actuator_outputs():
    global n

    if vehicle is not None:
        msg = vehicle.recv_match(blocking=False)
        if msg is not None:
            n += 1
            print(n, "<== IN: ", msg)

            # Check for actuator outputs message
            if msg.get_type() == 'HIL_ACTUATOR_CONTROLS':
                # Actuator outputs are usually in the 'controls' field of the message.
                actuator_outputs = msg.controls
                print("Received Actuator Outputs:", actuator_outputs)
            elif msg.get_type() == 'ACTUATOR_OUTPUT_STATUS':
                # Actuator outputs could also be in the 'output' field in ACTUATOR_OUTPUT_STATUS
                actuator_outputs = msg.output
                print("Received Actuator Outputs:", actuator_outputs)

# Create a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Drone 3D Trajectory')

# Set limits for your plot (change these as per your needs)
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([0, 20])

# Create a line object for trajectory
(line,) = ax.plot([state['x']], [state['y']], [state['z']], lw=2, c='r')

def update_plot():
    """Update the trajectory plot based on drone's position."""
    # Append new position
    x_data, y_data, z_data = line.get_data_3d()
    x_data.append(state['x'])
    y_data.append(state['y'])
    z_data.append(state['z'])

    # Set updated data
    line.set_data([x_data, y_data])
    line.set_3d_properties(z_data)

    # Draw updated plot
    plt.draw()
    plt.pause(0.001)  # Adjust for the desired refresh rate

### Simulation

try:

    while True:
        
        t__microseconds = time_absolute_microseconds - time_boot_microseconds
        t__seconds = t__microseconds / 1e6
        
        # Send and receive data with PX4 SITL
        send_heartbeat()
        send_gps()
        send_state_quaternion()
        send_sensor()
        
        actuators = receive_actuator_outputs()
        
        # Update the drone's physics state
        state = update_state_rk4(state, actuators, dt)
        update_plot()  # Update the trajectory plot

        
        # Sleep for the remainder of the 100 microseconds to ensure consistent loop frequency
        time.sleep(dt - (time.time() % dt))
        
        time_absolute_microseconds += 100
        
        # Optional: Stop condition (for example, if simulation time exceeds a limit)
        if t__seconds > SIMULATION_TIME:
            break

except KeyboardInterrupt:
    print("Simulation terminated by user.")

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    # Any cleanup or post-simulation tasks can be added here
    pass