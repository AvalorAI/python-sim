import time
from mavlink_connector import MavlinkConnector
from physics import VehiclePhysics
from visualizer import Visualizer
from gps import GPS

DT = 0.001
SIMULATION_TIME = 30
GROUND_LEVEL = 0
HOME_LAT = 51.4801492 * 1e7
HOME_LON = 5.3421784 * 1e7

physics = VehiclePhysics()
connector = MavlinkConnector()
visualyzer = Visualizer()
gps = GPS(HOME_LAT, HOME_LON)

# Set time
time_absolute_seconds = time.time()
time_absolute_microseconds = round(time_absolute_seconds * 1e6)
time_boot_microseconds = round(time_absolute_microseconds - 30e6)

try: 

    while True:
        
        # Update timer
        t__microseconds = time_absolute_microseconds - time_boot_microseconds
        t__seconds = t__microseconds / 1e6

        # Update MAVLink
        connector.send_heartbeat()
        connector.send_gps(time_absolute_microseconds, t__microseconds)
        connector.send_state_quaternion(time_absolute_microseconds, t__microseconds)
        connector.send_sensor(time_absolute_microseconds, t__microseconds)
        actuators = connector.receive_actuator_outputs()
        
        # Update the drone's physics state
        state = physics.update_state_rk4(state, actuators, DT, GROUND_LEVEL)

        # Update GPS
        gps_position = gps.get_gps_position(physics.state['x'], physics.state['y'])

        # Update barometer

        # Plot trajectory
        visualyzer.update_plot(state)

        # Sleep for the remainder of the 100 microseconds to ensure consistent loop frequency
        time.sleep(DT - (time.time() % DT))
            
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