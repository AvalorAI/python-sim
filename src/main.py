import time
from mavlink.mavlink_connector import MavlinkConnector
from vehicles.physics import IMU
#from util.visualizer import Visualizer
from sensors.gps import GPS
from vehicles.quad import Quadcopter

DT = 0.001
SIMULATION_TIME = 30000
GROUND_LEVEL = 0
HOME_LAT = 51.4801492 * 1e7
HOME_LON = 5.3421784 * 1e7

quad = Quadcopter()
physics = IMU()
gps = GPS(HOME_LAT, HOME_LON)
connector = MavlinkConnector()
# visualyzer = Visualizer(physics.state)

connector.setup_mavlink_connection(n=0)


# Set time
time_absolute_seconds = time.time()
time_absolute_microseconds = round(time_absolute_seconds * 1e6)
time_boot_microseconds = round(time_absolute_microseconds - 30e6)

try: 

    while True:
        
        # Update timer
        t__microseconds = time_absolute_microseconds - time_boot_microseconds
        t__seconds = t__microseconds / 1e6

        # Update sensor state
        state = physics.update_state_rk4(physics.state, physics.actuators, DT, GROUND_LEVEL)
        heading = physics.get_heading() # deg
        pressure = physics.pressure_from_altitude() # Hpa
        gps_position = gps.get_gps_position(physics.state['x'], physics.state['y'])

        # Update quadcopter state
        quad.velocity = [physics.state['v_x'], physics.state['v_y'], physics.state['v_z']] # cm/s
        quad.acceleration = [physics.state['acc_x'], physics.state['acc_y'], physics.state['acc_z']] # cm/s/s
        quad.attitude = physics.state['attitude_quaternion']
        quad.lat = gps_position[0] # 1e7
        quad.lon = gps_position[1] # 1e7
        quad.alt = physics.state['z'] # cm
        quad.heading = heading # deg
        quad.airspeed = physics.get_airspeed() # cm/s
        quad.angular_speed = physics.state['angular_v'] # rad/s
        quad.pressure = pressure # Hpa

        # Update MAVLink
        connector.send_heartbeat(time_absolute_seconds, connector.vehicle)
        connector.send_gps(time_absolute_microseconds, t__microseconds, quad)
        connector.send_state_quaternion(time_absolute_microseconds, t__microseconds, quad)
        connector.send_sensor(time_absolute_microseconds, t__microseconds, quad)
        actuators = connector.receive_actuator_outputs()

        # Plot trajectory
        # visualyzer.update_plot(state)

        # Sleep for the remainder of the 100 microseconds to ensure consistent loop frequency
        #time.sleep(DT - (time.time() % DT))
            
        time_absolute_microseconds += 100

        # Optional: Stop condition (for example, if simulation time exceeds a limit)
        if t__seconds > SIMULATION_TIME:
            break
    
    print("While loop ended")

except KeyboardInterrupt:
    print("Simulation terminated by user.")

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    # Any cleanup or post-simulation tasks can be added here
    pass