from random import random
drone = {}

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