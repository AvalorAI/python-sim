import time
import logging
from pymavlink import mavutil


# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

vehicle = None
n = 0

class MavlinkConnector:
    def __init__(self):
        self.vehicle = None

    def setup_mavlink_connection(self):

        global vehicle
        global n
        
        print("Waiting to connect...")
        vehicle = mavutil.mavlink_connection('tcpin:172.29.64.1:4560')
        
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
    

def send_data():
    whatever = True

time_absolute_seconds = time.time()
time_absolute_microseconds = round(time_absolute_seconds * 1e6)
time_boot_microseconds = round(time_absolute_microseconds - 30e6)

# Test the heartbeat function
connector = MavlinkConnector()
connector.setup_mavlink_connection()

while True:
    
    t__microseconds = time_absolute_microseconds - time_boot_microseconds
    t__seconds = t__microseconds / 1e6
    
    send_data()

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
        
        print (n, "OUT: --> HEARTBEAT {",
            "type :"                , the_type,
            ",",
            "autopilot :"           , autopilot,
            ",",
            "base_mode :"           , base_mode,
            ",",
            "custom_mode :"         , custom_mode,
            ",",
            "system_status :"       , system_status,
            ",",
            "mavlink_version :"     , mavlink_version,
        "}")

    if vehicle != None:
        msg = vehicle.recv_match(blocking = False)
        if msg != None:
            n += 1
            print(n, "<== IN: ", msg)
    
    
    time_absolute_microseconds += 100
