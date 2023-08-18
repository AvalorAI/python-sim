import time
import logging
from pymavlink import mavutil
from models import Drone
from visualizer import Drone3DMatplotlibVisualizer

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class MavlinkConnector:
    def __init__(self):
        self.vehicle = None

    def setup_mavlink_connection(self):
        try:
            self.vehicle = mavutil.mavlink_connection('tcpin:172.20.160.1:4560')
            self.vehicle.wait_heartbeat()
            logging.info("Heartbeat from system (system %u component %u)", self.vehicle.target_system, self.vehicle.target_component)
        except Exception as e:
            logging.error("Error establishing MAVLink connection: %s", str(e))
        return self.vehicle

    def send_mavlink_messages(self, drone):
            try:
                 
                # For demonstration, populate the hil_state_quaternion message
                hil_state_quaternion = {
                    'time_usec': int(time.time() * 1e6),
                    'attitude_quaternion': list(int(drone.quaternion.as_quat())),
                    'rollspeed': 0,  # Update if you implement rotational dynamics
                    'pitchspeed': 0,
                    'yawspeed': 0,
                    'lat': 0,  # Placeholder
                    'lon': 0,
                    'alt': int(drone.position[2]),
                    'vx': int(drone.velocity[0]),
                    'vy': int(drone.velocity[1]),
                    'vz': int(drone.velocity[2]),
                    'ind_airspeed': 0,  # Placeholder
                    'true_airspeed': 0,  # Placeholder
                    'xacc': 0,  # Placeholder for acceleration data
                    'yacc': 0,
                    'zacc': 0,
                }

                # Send the message
                self.vehicle.mav.hil_state_quaternion_send(**hil_state_quaternion)
                logging.debug("Sent hil_state_quaternion message: %s", str(hil_state_quaternion))

                # TODO: populate other messages

                hil_sensor = {}
                hil_gps = {}
                hil_optical_flow = {}
                hil_rc_inputs_raw = {}

                self.vehicle.mav.hil_sensor_send(**hil_sensor)
                self.vehicle.mav.hil_gps_send(**hil_gps)
                self.vehicle.mav.hil_optical_flow_send(**hil_optical_flow)
                self.vehicle.mav.hil_rc_inputs_raw_send(**hil_rc_inputs_raw)

            except Exception as e:
                logging.error("Error sending MAVLink message: %s", str(e))
            

    def receive_mavlink_messages(self, drone):
            try:
                # Listen for the HIL_ACTUATOR_CONTROLS message
                msg = self.vehicle.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=True)
                if msg:
                    logging.debug("Received HIL_ACTUATOR_CONTROLS message: %s", str(msg))
                    for i, act in enumerate(drone.actuators):
                        act.set_thrust(msg.controls[i] * drone.mass * drone.g / 4.0)
            except Exception as e:
                 logging.error("Error receiving MAVLink message: %s", str(e))