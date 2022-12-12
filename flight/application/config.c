#include <UAVWare.h>
#include "config.h"


// run time copy of persistant storage data:
config_t config;

// parameter index struct to help get/set parameters via MAVlink
struct params_s {
  const char * paramName;
  uintptr_t offset;
  MAV_PARAM_TYPE mav_type;
  bool read_write;
};


void config_set_defaults( void )
{

  config.version = 55;

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
  config.channel_1_fs = 1000;    //thro
  config.channel_2_fs = 1500;    //ail
  config.channel_3_fs = 1500;    //elev
  config.channel_4_fs = 1500;    //rudd
  config.channel_5_fs = 2000;    //gear, greater than 1500 = throttle cut
  config.channel_6_fs = 2000;    //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
  config.B_madgwick = 0.04;      //Madgwick filter parameter
  config.B_accel = 0.14;         //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
  config.B_gyro = 0.1;           //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
  config.B_mag = 1.0;            //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
  config.MagErrorX = 0.0;
  config.MagErrorY = 0.0;
  config.MagErrorZ = 0.0;
  config.MagScaleX = 1.0;
  config.MagScaleY = 1.0;
  config.MagScaleZ = 1.0;

//Controller parameters (take note of defaults before modifying!):
  config.i_limit = 25.0;         //Integrator saturation level, mostly for safety (default 25.0)
  config.maxRoll = 30.0;         //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
  config.maxPitch = 30.0;        //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
  config.maxYaw = 160.0;         //Max yaw rate in deg/sec

  config.Kp_roll_angle = 0.2;    //Roll P-gain - angle mode
  config.Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
  config.Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
  config.B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
  config.Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
  config.Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
  config.Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
  config.B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

  config.Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
  config.Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
  config.Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  config.Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
  config.Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
  config.Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

  config.Kp_yaw = 0.3;           //Yaw P-gain
  config.Ki_yaw = 0.05;          //Yaw I-gain
  config.Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

  config.z_Save = 0;             // Flag to save setup (default to 0, set to 1 to save)

}


// typedef struct params_s params_t;
static const struct params_s param_arr[] = {
[ 0 ] = { .paramName  = "version" , .offset = offsetof( config_t, version ),  .mav_type = MAV_PARAM_TYPE_UINT8  , .read_write = false   },
                                  
                                  
[ 1 ] = { .paramName  = "channel_1_fs"  , .offset = offsetof( config_t, channel_1_fs  ),  .mav_type = MAV_PARAM_TYPE_UINT32 , .read_write = true    },
[ 2 ] = { .paramName  = "channel_2_fs"  , .offset = offsetof( config_t, channel_2_fs  ),  .mav_type = MAV_PARAM_TYPE_UINT32 , .read_write = true    },
[ 3 ] = { .paramName  = "channel_3_fs"  , .offset = offsetof( config_t, channel_3_fs  ),  .mav_type = MAV_PARAM_TYPE_UINT32 , .read_write = true    },
[ 4 ] = { .paramName  = "channel_4_fs"  , .offset = offsetof( config_t, channel_4_fs  ),  .mav_type = MAV_PARAM_TYPE_UINT32 , .read_write = true    },
[ 5 ] = { .paramName  = "channel_5_fs"  , .offset = offsetof( config_t, channel_5_fs  ),  .mav_type = MAV_PARAM_TYPE_UINT32 , .read_write = true    },
[ 6 ] = { .paramName  = "channel_6_fs"  , .offset = offsetof( config_t, channel_6_fs  ),  .mav_type = MAV_PARAM_TYPE_UINT32 , .read_write = true    },
                                  
                                  
[ 7 ] = { .paramName  = "B_madgwick"  , .offset = offsetof( config_t, B_madgwick  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 8 ] = { .paramName  = "B_accel" , .offset = offsetof( config_t, B_accel ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 9 ] = { .paramName  = "B_gyro"  , .offset = offsetof( config_t, B_gyro  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 10  ] = { .paramName  = "B_mag" , .offset = offsetof( config_t, B_mag ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
                                  
                                  
[ 11  ] = { .paramName  = "MagErrorX" , .offset = offsetof( config_t, MagErrorX ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 12  ] = { .paramName  = "MagErrorY" , .offset = offsetof( config_t, MagErrorY ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 13  ] = { .paramName  = "MagErrorZ" , .offset = offsetof( config_t, MagErrorZ ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 14  ] = { .paramName  = "MagScaleX" , .offset = offsetof( config_t, MagScaleX ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 15  ] = { .paramName  = "MagScaleY" , .offset = offsetof( config_t, MagScaleY ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 16  ] = { .paramName  = "MagScaleZ" , .offset = offsetof( config_t, MagScaleZ ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
                                  
                                  
[ 17  ] = { .paramName  = "i_limit" , .offset = offsetof( config_t, i_limit ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 18  ] = { .paramName  = "maxRoll" , .offset = offsetof( config_t, maxRoll ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 19  ] = { .paramName  = "maxPitch"  , .offset = offsetof( config_t, maxPitch  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 20  ] = { .paramName  = "maxYaw " , .offset = offsetof( config_t, maxYaw  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
                                  
[ 21  ] = { .paramName  = "Kp_roll_angle" , .offset = offsetof( config_t, Kp_roll_angle ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 22  ] = { .paramName  = "Ki_roll_angle" , .offset = offsetof( config_t, Ki_roll_angle ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 23  ] = { .paramName  = "Kd_roll_angle" , .offset = offsetof( config_t, Kd_roll_angle ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 24  ] = { .paramName  = "B_loop_roll" , .offset = offsetof( config_t, B_loop_roll ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 25  ] = { .paramName  = "Kp_pitch_angle"  , .offset = offsetof( config_t, Kp_pitch_angle  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 26  ] = { .paramName  = "Ki_pitch_angle"  , .offset = offsetof( config_t, Ki_pitch_angle  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 27  ] = { .paramName  = "Kd_pitch_angle"  , .offset = offsetof( config_t, Kd_pitch_angle  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 28  ] = { .paramName  = "B_loop_pitch"  , .offset = offsetof( config_t, B_loop_pitch  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
                                  
[ 29  ] = { .paramName  = "Kp_roll_rate"  , .offset = offsetof( config_t, Kp_roll_rate  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 30  ] = { .paramName  = "Ki_roll_rate"  , .offset = offsetof( config_t, Ki_roll_rate  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 31  ] = { .paramName  = "Kd_roll_rate"  , .offset = offsetof( config_t, Kd_roll_rate  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 32  ] = { .paramName  = "Kp_pitch_rate" , .offset = offsetof( config_t, Kp_pitch_rate ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 33  ] = { .paramName  = "Ki_pitch_rate" , .offset = offsetof( config_t, Ki_pitch_rate ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 34  ] = { .paramName  = "Kd_pitch_rate" , .offset = offsetof( config_t, Kd_pitch_rate ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
                                  
[ 35  ] = { .paramName  = "Kp_yaw"  , .offset = offsetof( config_t, Kp_yaw  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 36  ] = { .paramName  = "Ki_yaw"  , .offset = offsetof( config_t, Ki_yaw  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
[ 37  ] = { .paramName  = "Kd_yaw"  , .offset = offsetof( config_t, Kd_yaw  ),  .mav_type = MAV_PARAM_TYPE_REAL32 , .read_write = true    },
                                  
[ 38  ] = { .paramName  = "z_Save"  , .offset = offsetof( config_t, z_Save  ),  .mav_type = MAV_PARAM_TYPE_UINT8  , .read_write = true    },
};

uint32_t get_sizeof_param_index( void )
{
  return sizeof param_arr / sizeof param_arr[0];
}

const char * get_param_name( uint32_t index )
{
  return param_arr[ index ].paramName;
}

uintptr_t get_param_offset( uint32_t index )
{
  return param_arr[ index ].offset;
}

MAV_PARAM_TYPE get_param_mav_type( uint32_t index )
{
  return param_arr[ index ].mav_type;
}

bool get_param_read_write( uint32_t index )
{
  return param_arr[ index ].read_write;
}

bool check_param_match( const char * paramName, char * key )
{
  for ( uint32_t j = 0; paramName[j] != '\0'; j++ ) {
    if ( ( ( char ) ( paramName[j] ) ) != ( char ) ( key[j] ) ) {
      return false;
    }
  }
  return true;
}

// https://ardupilot.org/dev/docs/mavlink-get-set-params.html
uint32_t get_param_index_from_id( char * param_id )
{
  uint32_t cnt = get_sizeof_param_index() - 1;
  for ( uint32_t i = 0; i <= cnt; ++i ) {
    if ( check_param_match( param_arr[i].paramName, param_id ) ) {
      return i;
    }
  }
  return -1;
}