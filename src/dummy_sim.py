# Author:   Jean Dupond
# Version:  1.0.0
# 
# Dummy simulator for PX4 SITL to test communication with
# PX4. This script simulates a drone at rest on the ground.
# It reproduces jMAVSim (make px4_sitl jmavsim) behavior 
# before the mission starts.
# 
# HOW TO:
# 1- Launch PX4 SITL, eg.:
#   make px4_sitl none_iris
# 2- In another terminal, launch this script.
# 3- Launch QGroundControl.
# 
# The drone should appear. You should be able to upload a
# mission and switch to flight mode. The drone should arm 
# and PX4 should start sending non-zero actuator controls
# (visible in the output).
# 

import time
import atexit
from random import random
from pymavlink import mavutil

# =========================================================
# =========================================================
# =========================================================

vehicle = None

def close():
    if vehicle != None:
        vehicle.close()
atexit.register(close)

drone = {}

n = 0

# =========================================================
# =========================================================
# =========================================================

def crandom():
    return random()-0.5

def Update():
    
    #        f:float --------------------> float
    #        i: int  --------------------> round  SI value         noise           unit conver.
    #        |                               |       |               |                  |
    #        v                               v       v               v                  v
    # drone['i_name__unit']              = round((   0           +random()*0.1       )*1000)
    
    
    # HIL_GPS Data
    
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
    
    
    # HIL_SENSOR Data
    
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
    
    
    # HIL_STATE_QUATERNION  Data
    
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
    

def Connect():
    
    global vehicle
    global n
    
    print("Waiting to connect...")
    vehicle = mavutil.mavlink_connection('tcpin:172.20.160.1:4560')
    
    msg = vehicle.recv_match(blocking = True)
    if msg.get_type() != "COMMAND_LONG":
        raise Exception("error")
    n += 1
    print(n, "<==", msg)
    
    msg = vehicle.recv_match(blocking = True)
    if msg.get_type() != "HEARTBEAT":
        raise Exception("error")
    n += 1
    print(n, "<==", msg)
    

# =========================================================
# =========================================================
# =========================================================

Connect()

t_abs__s    = time.time()

t_abs__us   = round(t_abs__s * 1e6)
t_boot__us  = round(t_abs__us - 30e6)

while True:
    
    t__us = t_abs__us - t_boot__us
    t__s = t__us / 1e6
    
    Update()
    
    
    #
    # SYSTEM_TIME
    #
    
    if t__us % 4000000 == 0:
        n += 1
        
        since_boot__us = t_abs__us - t_boot__us
        since_boot__ms = round(since_boot__us / 1000)
        
        time_unix_usec      = t_abs__us
        time_boot_ms        = since_boot__ms
        
        if vehicle != None:
            vehicle.mav.system_time_send(
                time_unix_usec  = time_unix_usec        , # Timestamp (UNIX epoch time). [us] (type:uint64_t)
                time_boot_ms    = time_boot_ms          , # Timestamp (time since system boot). [ms] (type:uint32_t)
            )
        
        print (n, "--> SYSTEM_TIME {",
            "time_unix_usec :", time_unix_usec,
            ",",
            "time_boot_ms :", time_boot_ms,
        "}")
    
    
    #
    # HEARTBEAT
    #
    
    if t__us % 1000000 == 0:
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
        
        print (n, "--> HEARTBEAT {",
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
        
    
    
    #
    # HIL_GPS
    #
    
    if t__us % 52000 == 0:
        n += 1
        
        time_usec           = t_abs__us                 # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
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
        
        print (n, "--> HIL_GPS {",
            "time_usec :"           , time_usec,
            ",",
            "fix_type :"            , fix_type,
            ",",
            "lat :"                 , lat,
            ",",
            "lon :"                 , lon,
            ",",
            "alt :"                 , alt,
            ",",
            "eph :"                 , eph,
            ",",
            "epv :"                 , epv,
            ",",
            "vel :"                 , vel,
            ",",
            "vn :"                  , vn,
            ",",
            "ve :"                  , ve,
            ",",
            "vd :"                  , vd,
            ",",
            "cog :"                 , cog,
            ",",
            "satellites_visible :"  , satellites_visible,
            ",",
            "id :"                  , the_id,
            ",",
            "yaw :"                 , yaw,
        "}")
        
    
    
    #
    # HIL_STATE_QUATERNION
    #
    
    if t__us % 8000 == 0:
        n += 1
        
        time_usec           = t_abs__us                         # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
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
        
        print (n, "--> HIL_STATE_QUATERNION {",
            "time_usec :"           , time_usec,
            ",",
            "attitude_quaternion :" , attitude_quaternion,
            ",",
            "rollspeed :"           , rollspeed,
            ",",
            "pitchspeed :"          , pitchspeed,
            ",",
            "yawspeed :"            , yawspeed,
            ",",
            "lat :"                 , lat,
            ",",
            "lon :"                 , lon,
            ",",
            "alt :"                 , alt,
            ",",
            "vx :"                  , vx,
            ",",
            "vy :"                  , vy,
            ",",
            "vz :"                  , vz,
            ",",
            "ind_airspeed :"        , ind_airspeed,
            ",",
            "true_airspeed :"       , true_airspeed,
            ",",
            "xacc :"                , xacc,
            ",",
            "yacc :"                , yacc,
            ",",
            "zacc :"                , zacc,
        "}")
        
    
    
    #
    # HIL_SENSOR
    #
    
    if t__us % 4000 == 0:
        n += 1
        
        time_usec           = t_abs__us                     # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
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
        
        print (n, "--> HIL_SENSOR {",
            "time_usec :"       , time_usec,
            ",",
            "xacc :"            , xacc,
            ",",
            "yacc :"            , yacc,
            ",",
            "zacc :"            , zacc,
            ",",
            "xgyro :"           , xgyro,
            ",",
            "ygyro :"           , ygyro,
            ",",
            "zgyro :"           , zgyro,
            ",",
            "xmag :"            , xmag,
            ",",
            "ymag :"            , ymag,
            ",",
            "zmag :"            , zmag,
            ",",
            "abs_pressure :"    , abs_pressure,
            ",",
            "diff_pressure :"   , diff_pressure,
            ",",
            "pressure_alt :"    , pressure_alt,
            ",",
            "temperature :"     , temperature,
            ",",
            "fields_updated :"  , fields_updated,
            ",",
            "id :"              , the_id,
        "}")
        
    
    if vehicle != None:
        msg = vehicle.recv_match(blocking = False)
        if msg != None:
            n += 1
            print(n, "<==", msg)
    
    
    t_abs__us += 100
    #time.sleep(1e-7)
    