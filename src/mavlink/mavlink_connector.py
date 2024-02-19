import logging
import numpy as np
from pymavlink import mavutil
from random import random
from vehicles.quad import Quadcopter

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

UINT16_MAX = 0xFFFF

class MavlinkConnector:
    
    def __init__(self):
        self.vehicle = None
        self.n = 0

    def crandom():
        return random()-0.5
    
    def setup_mavlink_connection(self, n):

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


    def send_heartbeat(self, t__seconds, vehicle):
        
        if t__seconds % 1000000 == 0:

            self.n += 1
                
            if vehicle != None:
                vehicle.mav.heartbeat_send(
                    type                = 0, # Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type. (type:uint8_t, values:MAV_TYPE)
                    autopilot           = 0, # Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
                    base_mode           = 0, # System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
                    custom_mode         = 0, # A bitfield for use for autopilot-specific flags (type:uint32_t)
                    system_status       = 0, # System status flag. (type:uint8_t, values:MAV_STATE)
                    mavlink_version     = 3, # uint8_t_mavlink_version (type:uint8_t)
                )
            
            print(self.n, "OUT: --> HEARTBEAT")


    def send_gps(self, time_absolute_microseconds, t__microseconds, quad: Quadcopter):

        if t__microseconds % 52000 == 0:

            self.n += 1
               
            if self.vehicle != None:
                self.vehicle.mav.hil_gps_send(
                    time_usec           = np.int64(time_absolute_microseconds), # Timestamp [us] (type:uint64_t)
                    fix_type            = np.int8(3), # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. (type:uint8_t)
                    lat                 = np.int32(quad.lat), # Latitude (WGS84) [degE7] (type:int32_t)
                    lon                 = np.int32(quad.lon), # Longitude (WGS84) [degE7] (type:int32_t)
                    alt                 = np.int32(quad.alt * 100), # Altitude (MSL). Positive for up. [mm] (type:int32_t)
                    eph                 = UINT16_MAX, # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
                    epv                 = UINT16_MAX, # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
                    vel                 = np.int16(65535), # GPS ground speed. If unknown, set to: 65535 [cm/s] (type:uint16_t)
                    vn                  = np.int16(quad.vy *-1), # GPS velocity in north direction in earth-fixed NED frame [cm/s] (type:int16_t)
                    ve                  = np.int16(quad.vx), # GPS velocity in east direction in earth-fixed NED frame [cm/s] (type:int16_t)
                    vd                  = np.int16(quad.vz * -1), # GPS velocity in down direction in earth-fixed NED frame [cm/s] (type:int16_t)
                    cog                 = np.int16(65535), # Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
                    satellites_visible  = np.int8(10), # Number of satellites visible. If unknown, set to 255 (type:uint8_t)
                    id                  = np.int8(0), # GPS ID (zero indexed). Used for multiple GPS inputs (type:uint8_t)
                    yaw                 = np.int16(quad.heading * 100), # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north [cdeg] (type:uint16_t)
                )

        print(self.n, "OUT: --> GPS")


    def send_state_quaternion(self, time_absolute_microseconds, t__microseconds, quad: Quadcopter):

        if t__microseconds % 8000 == 0:
            self.n += 1
            
            if self.vehicle != None:
                self.vehicle.mav.hil_state_quaternion_send(
                    time_usec           = np.int64(time_absolute_microseconds), # Timestamp [us] (type:uint64_t)
                    attitude_quaternion = quad.attitude, # Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation) (type:float)
                    rollspeed           = float(quad.angular_speed[0]), # Body frame roll / phi angular speed [rad/s] (type:float)
                    pitchspeed          = float(quad.angular_speed[1]), # Body frame pitch / theta angular speed [rad/s] (type:float)
                    yawspeed            = float(quad.angular_speed[2]), # Body frame yaw / psi angular speed [rad/s] (type:float)
                    lat                 = np.int32(quad.lat), # Latitude [degE7] (type:int32_t)
                    lon                 = np.int32(quad.lon), # Longitude [degE7] (type:int32_t)
                    alt                 = np.int32(quad.alt * 100), # Altitude [mm] (type:int32_t)
                    vx                  = np.int16(quad.velocity[0]), # Ground X Speed (Latitude) [cm/s] (type:int16_t)
                    vy                  = np.int16(quad.velocity[1]), # Ground Y Speed (Longitude) [cm/s] (type:int16_t)
                    vz                  = np.int16(quad.velocity[2]), # Ground Z Speed (Altitude) [cm/s] (type:int16_t)
                    ind_airspeed        = np.int16(quad.airspeed), # Indicated airspeed [cm/s] (type:uint16_t)
                    true_airspeed       = np.int16(quad.airspeed), # True airspeed [cm/s] (type:uint16_t)
                    xacc                = np.int16((quad.acceleration[0] / 981) * 1000), # X acceleration [mG] (type:int16_t)
                    yacc                = np.int16((quad.acceleration[1] / 981) * 1000), # Y acceleration [mG] (type:int16_t)
                    zacc                = np.int16((quad.acceleration[2] / 981) * 1000), # Z acceleration [mG] (type:int16_t)
                )
            
            print (self.n, "--> HIL_STATE_QUATERNION")


    def send_sensor(self, time_absolute_microseconds, t__microseconds, quad: Quadcopter):

        if t__microseconds % 4000 == 0:
            
            self.n += 1
            
            if self.vehicle != None:
                self.vehicle.mav.hil_sensor_send(
                    time_usec           = np.int64(time_absolute_microseconds), # Timestamp [us] (type:uint64_t)
                    xacc                = float(quad.acceleration[0] / 100), # X acceleration [m/s/s] (type:float)
                    yacc                = float(quad.acceleration[1] / 100), # Y acceleration [m/s/s] (type:float)
                    zacc                = float(quad.acceleration[2] / 100), # Z acceleration [m/s/s] (type:float)
                    xgyro               = float(quad.angular_speed[0]), # Angular speed around X axis in body frame [rad/s] (type:float)
                    ygyro               = float(quad.angular_speed[1]), # Angular speed around Y axis in body frame [rad/s] (type:float)
                    zgyro               = float(quad.angular_speed[2]), # Angular speed around X axis in body frame [rad/s] (type:float)
                    xmag                = float(quad.xmag), # X Magnetic field [gauss] (type:float)
                    ymag                = float(quad.ymag), # Y Magnetic field [gauss] (type:float)
                    zmag                = float(quad.zmag), # Z Magnetic field [gauss] (type:float)
                    abs_pressure        = float(quad.pressure), # Absolute pressure [hPa] (type:float)
                    diff_pressure       = float(quad.pressure), # Differential pressure (airspeed) [hPa] (type:float)
                    pressure_alt        = float(quad.alt), # Altitude calculated from pressure (type:float)
                    temperature         = float(40.0), # Temperature [degC] (type:float)
                    fields_updated      = np.int32(7167), # Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)
                    id                  = np.int8(0) # Sensor ID (zero indexed). Used for multiple sensor inputs (type:uint8_t)
                )
            
            print (self.n, "--> HIL_SENSOR ")

    def receive_actuator_outputs(self):

        if self.vehicle is not None:
            msg = self.vehicle.recv_match(blocking=False)
            if msg is not None:
                self.n += 1
                print(self.n, "<== IN: ", msg)

                # Check for actuator outputs message
                if msg.get_type() == 'HIL_ACTUATOR_CONTROLS':
                    # Actuator outputs are usually in the 'controls' field of the message.
                    actuator_outputs = msg.controls
                    print("Received Actuator Outputs:", actuator_outputs)
                    return actuator_outputs
                
                elif msg.get_type() == 'ACTUATOR_OUTPUT_STATUS':
                    # Actuator outputs could also be in the 'output' field in ACTUATOR_OUTPUT_STATUS
                    actuator_outputs = msg.output
                    print("Received Actuator Outputs:", actuator_outputs)
                    return actuator_outputs


