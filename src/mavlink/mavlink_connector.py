import logging
from pymavlink import mavutil
from random import random

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

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
            
            print (self.n, "OUT: --> HEARTBEAT")


    def send_gps(self, time_absolute_microseconds, t__microseconds, drone):

        if t__microseconds % 52000 == 0:

            self.n += 1
            
            time_usec           = time_absolute_microseconds# Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
            fix_type            = 3                         # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. (type:uint8_t)
            lat                 = drone['i_lat__degE7']     # Latitude (WGS84) [degE7] (type:int32_t)
            lon                 = drone['i_lon__degE7']     # Longitude (WGS84) [degE7] (type:int32_t)
            alt                 = drone['i_alt__mm']        # Altitude (MSL). Positive for up. [mm] (type:int32_t)
            eph                 = drone['i_eph__cm']        # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
            epv                 = drone['i_epv__cm']        # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
            vel                 = drone['i_vel__cm/s']      # GPS ground speed. If unknown, set to: 65535 [cm/s] (type:uint16_t)
            vn                  = drone['i_vn__cm/s']       # GPS velocity in north direction in earth-fixed NED frame [cm/s] (type:int16_t)
            ve                  = drone['i_ve__cm/s']       # GPS velocity in east direction in earth-fixed NED frame [cm/s] (type:int16_t)
            vd                  = drone['i_vd__cm/s']       # GPS velocity in down direction in earth-fixed NED frame [cm/s] (type:int16_t)
            cog                 = drone['i_cog__cdeg']      # Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
            satellites_visible  = 10                        # Number of satellites visible. If unknown, set to 255 (type:uint8_t)
            the_id              = 0                         # GPS ID (zero indexed). Used for multiple GPS inputs (type:uint8_t)
            yaw                 = 0                         # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north [cdeg] (type:uint16_t)
            
            if self.vehicle != None:
                self.vehicle.mav.hil_gps_send(
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

        print (self.n, "OUT: --> GPS")


    def send_state_quaternion(self, time_absolute_microseconds, t__microseconds, drone):

        if t__microseconds % 8000 == 0:
            n += 1
            
            time_usec           = time_absolute_microseconds        # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
            attitude_quaternion = drone['f_attitude_quaternion__1'] # Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation) (type:float)
            rollspeed           = drone['f_rollspeed__rad/s']       # Body frame roll / phi angular speed [rad/s] (type:float)
            pitchspeed          = drone['f_pitchspeed__rad/s']      # Body frame pitch / theta angular speed [rad/s] (type:float)
            yawspeed            = drone['f_yawspeed__rad/s']        # Body frame yaw / psi angular speed [rad/s] (type:float)
            lat                 = drone['i_lat__degE7']             # Latitude [degE7] (type:int32_t)
            lon                 = drone['i_lon__degE7']             # Longitude [degE7] (type:int32_t)
            alt                 = drone['i_alt__mm']                # Altitude [mm] (type:int32_t)
            vx                  = drone['i_vx__cm/s']               # Ground X Speed (Latitude) [cm/s] (type:int16_t)
            vy                  = drone['i_vy__cm/s']               # Ground Y Speed (Longitude) [cm/s] (type:int16_t)
            vz                  = drone['i_vz__cm/s']               # Ground Z Speed (Altitude) [cm/s] (type:int16_t)
            ind_airspeed        = drone['i_ind_airspeed__cm/s']     # Indicated airspeed [cm/s] (type:uint16_t)
            true_airspeed       = drone['i_true_airspeed__cm/s']    # True airspeed [cm/s] (type:uint16_t)
            xacc                = drone['i_xacc__mG']               # X acceleration [mG] (type:int16_t)
            yacc                = drone['i_yacc__mG']               # Y acceleration [mG] (type:int16_t)
            zacc                = drone['i_zacc__mG']               # Z acceleration [mG] (type:int16_t)
            
            if self.vehicle != None:
                self.vehicle.mav.hil_state_quaternion_send(
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
            
            print (self.n, "--> HIL_STATE_QUATERNION")


    def send_sensor(self, time_absolute_microseconds, t__microseconds, drone):

    

        if t__microseconds % 4000 == 0:
            self.n += 1
            
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
            
            if self.vehicle != None:
                self.vehicle.mav.hil_sensor_send(
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


