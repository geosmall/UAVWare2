// UAVWare Flight Controller Library - dRehmFlight based application
// Version: 0.1

#include <UAVWare.h>
#include "printf.h"
#include "config.h"

//========================================================================================================================//
//                                                 USER Function Declarations                                             //
//========================================================================================================================//

void getIMUdata();
void calculate_IMU_error();
void Madgwick( float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq );
void Madgwick6DOF( float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq );
void getDesState();
void controlANGLE();
void controlMixer();
void scaleCommands();
void getCommands();
void failSafe();
void throttleCut();

void loopBlink();
float invSqrt( float x );
void setupBlink( int numBlinks, int upTime, int downTime );
void loopRate( int freq );
void printRadioData();
void printGyroData();
void printAccelData();

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

/* Uncomment only one RX protocol */
#define RCVR_PROTOCOL UVOS_RCVR_CHANNELGROUPS_IBUS //default
// #define RCVR_PROTOCOL UVOS_RCVR_CHANNELGROUPS_SBUS

/* Uncomment only one IMU */
#define USE_MPU_6DOF //default
// #define USE_MPU_9DOF

/* Uncomment only one full scale gyro range (deg/sec) */
#define GYRO_250DPS //default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

/* Uncomment only one full scale accelerometer range (G's) */
#define ACCEL_2G //default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

/* Uncomment only one mpu low pass filter setting (Hz) */
#define LOWPASS_256_HZ // default
// #define LOWPASS_188_HZ
// #define LOWPASS_98_HZ
// #define LOWPASS_42_HZ
// #define LOWPASS_20_HZ
// #define LOWPASS_10_HZ
// #define LOWPASS_5_HZ

/* Main loop frequency in Hertz */
#define LOOP_FREQ_HZ 1000

/* Number of sample loops for imu calibration */
#define IMU_CAL_ITERATION_LOOPS 12000
#define ATTITUDE_CAL_ITERATION_LOOPS 10000

//========================================================================================================================//

/* Setup gyro and accel full scale value selection and scale factor */

#if defined GYRO_250DPS
#define GYRO_SCALE UVOS_GYRO_FS_250DPS_BITS
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE UVOS_GYRO_FS_500DPS_BITS
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE UVOS_GYRO_FS_1000DPS_BITS
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE UVOS_GYRO_FS_2000DPS_BITS
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE UVOS_ACCEL_FS_2G_BITS
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE UVOS_ACCEL_FS_4G_BITS
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE UVOS_ACCEL_FS_8G_BITS
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE UVOS_ACCEL_FS_16G_BITS
#define ACCEL_SCALE_FACTOR 2048.0
#endif

#if defined  LOWPASS_256_HZ
#define FILTER_SETTING UVOS_LOWPASS_256_HZ_BITS
#elif defined LOWPASS_188_HZ
#define FILTER_SETTING UVOS_LOWPASS_188_HZ_BITS
#elif defined LOWPASS_98_HZ
#define FILTER_SETTING UVOS_LOWPASS_98_HZ_BITS
#elif defined LOWPASS_42_HZ
#define FILTER_SETTING UVOS_LOWPASS_42_HZ_BITS
#elif defined LOWPASS_20_HZ
#define FILTER_SETTING UVOS_LOWPASS_20_HZ_BITS
#elif defined LOWPASS_10_HZ
#define FILTER_SETTING UVOS_LOWPASS_10_HZ_BITS
#elif defined LOWPASS_5_HZ
#define FILTER_SETTING UVOS_LOWPASS_5_HZ_BITS
#endif

//========================================================================================================================//

// UAVWare NOTE: C function return status convention used ( 0 = success, non-zero = failure )

//========================================================================================================================//

//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

// #if defined USE_SBUS_RX
//   SBUS sbus(Serial5);
//   uint16_t sbusChannels[16];
//   bool sbusFailSafe;
//   bool sbusLostFrame;
// #endif
// #if defined USE_DSM_RX
//   DSM1024 DSM;
// #endif
uint16_t rc_command_uSec[ UVOS_RCVR_MAX_CHANNELS ];

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
// float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
// int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
// float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
// int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;
float m1_command_summed, m2_command_summed, m3_command_summed, m4_command_summed, m5_command_summed, m6_command_summed;
float m1_command, m2_command, m3_command, m4_command, m5_command, m6_command;
float s1_command_summed, s2_command_summed, s3_command_summed, s4_command_summed, s5_command_summed, s6_command_summed, s7_command_summed;
float s1_command, s2_command, s3_command, s4_command, s5_command, s6_command, s7_command;


//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//

void setup( void )
{

  /* Brings up System using CMSIS functions, initializes periph clock, gpio pins. */
  UVOS_SYS_Init();

  if ( UVOS_Board_Init() ) {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_DEBUG_Panic( "System initialization Error\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  } else {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_COM_SendString( UVOS_COM_DEBUG, "System initialized\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  }

  /* Set config struct default values */
  config_set_defaults();

  // LED on to signal startup & not to disturb vehicle during IMU calibration
  UW_led_heartbeat_on();
  UW_led_alarm_off();

  UW_time_delay_us( 10000 );

  // Set IMU full-scale ranges for accel and gyro
  mpu_set_imu_ranges( GYRO_SCALE, ACCEL_SCALE, FILTER_SETTING );
  mpu_set_speed_fast();


  // Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = config.channel_1_fs;
  channel_2_pwm = config.channel_2_fs;
  channel_3_pwm = config.channel_3_fs;
  channel_4_pwm = config.channel_4_fs;
  channel_5_pwm = config.channel_5_fs;
  channel_6_pwm = config.channel_6_fs;

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level
  calculate_IMU_error();

  UW_time_delay_us( 10000 );

  // Arm servo channels
  // servo1.write(0); //command servo angle from 0-180 degrees (1000 to 2000 PWM)
  // servo2.write(0);
  // servo3.write(0);
  // servo4.write(0);
  // servo5.write(0);
  // servo6.write(0);
  // servo7.write(0);

  UW_time_delay_us( 10000 );

  // Arm motors
  m1_command = 0.0f; //command ESC (from 0=low to 1=high)
  m2_command = 0.0f;
  m3_command = 0.0f;
  m4_command = 0.0f;
  m5_command = 0.0f;
  m6_command = 0.0f;
  // commandMotors();

  // Indicate entering main loop with 3 quick blinks
  setupBlink( 3, 160, 70 ); //numBlinks, upTime (ms), downTime (ms)

  // If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  // calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//

void loop()
{
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = ( current_time - prev_time ) / 1000000.0;

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  // printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  // printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  // printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  // printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  // printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  // printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  // printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  // printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  // printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

  // Get vehicle state
  getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick( GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt ); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  // Compute desired state
  getDesState(); // Convert raw commands to normalized values based on saturated control limits

  // PID Controller - SELECT ONE:
  controlANGLE(); // Stabilize on angle setpoint
  // controlANGLE2(); // Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  // controlRATE(); // Stabilize on rate setpoint

  // Actuator mixing and scaling to PWM values
  controlMixer(); // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  // Throttle cut check
  throttleCut(); // Directly sets motor commands to low based on state of ch5

  // Command actuators
  // commandMotors(); // Sends command pulses to each motor pin using OneShot125 protocol
  // servo1.write( s1_command_PWM ); //Writes PWM value to servo object
  // servo2.write( s2_command_PWM );
  // servo3.write( s3_command_PWM );
  // servo4.write( s4_command_PWM );
  // servo5.write( s5_command_PWM );
  // servo6.write( s6_command_PWM );
  // servo7.write( s7_command_PWM );

  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands
  failSafe(); // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  // Regulate loop rate
  loopRate( 2000 ); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

void controlMixer()
{
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_summed and sX_command scaled variables are used in scaleCommands()
   * in preparation to be sent to the motor ESCs and servos.
   */
  // Quad mixing
  // m1 = front left, m2 = front right, m3 = back right, m4 = back left
  m1_command_summed = thro_des - pitch_PID + roll_PID + yaw_PID;
  m2_command_summed = thro_des - pitch_PID - roll_PID - yaw_PID;
  m3_command_summed = thro_des + pitch_PID - roll_PID + yaw_PID;
  m4_command_summed = thro_des + pitch_PID + roll_PID - yaw_PID;
  m5_command_summed = 0.0f;
  m6_command_summed = 0.0f;

  // 0.5 is centered servo, 0 is zero throttle if connecting to ESC for conventional PWM, 1 is max throttle
  s1_command_summed = 0.0f;
  s2_command_summed = 0.0f;
  s3_command_summed = 0.0f;
  s4_command_summed = 0.0f;
  s5_command_summed = 0.0f;
  s6_command_summed = 0.0f;
  s7_command_summed = 0.0f;

  // Example use of the linear fader for float type variables. Linearly interpolate between minimum and maximum values for Kp_pitch_rate variable based on state of channel 6:
  /*
  if (channel_6_pwm > 1500){ // go to max specified value in 5.5 seconds
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 5.5, 1, 2000); // parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
  }
  if (channel_6_pwm < 1500) { // go to min specified value in 2.5 seconds
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 2.5, 0, 2000); // parameter, minimum value, maximum value, fadeTime, state (0 min or 1 max), loop frequency
  }
  */
}
void getIMUdata()
{
  // DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ.
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */

#if defined USE_MPU_9DOF
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
  mpu_read_imu9( &AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ );
#else
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  mpu_read_imu( &AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ );
#endif

  // Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; // G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  // LP filter accelerometer data
  AccX = ( 1.0 - config.B_accel ) * AccX_prev + config.B_accel * AccX;
  AccY = ( 1.0 - config.B_accel ) * AccY_prev + config.B_accel * AccY;
  AccZ = ( 1.0 - config.B_accel ) * AccZ_prev + config.B_accel * AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  // Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; // deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  // LP filter gyro data
  GyroX = ( 1.0 - config.B_gyro ) * GyroX_prev + config.B_gyro * GyroX;
  GyroY = ( 1.0 - config.B_gyro ) * GyroY_prev + config.B_gyro * GyroY;
  GyroZ = ( 1.0 - config.B_gyro ) * GyroZ_prev + config.B_gyro * GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

#if defined USE_MPU_9DOF
  // Magnetometer
  MagX = MgX / 6.0; // uT
  MagY = MgY / 6.0;
  MagZ = MgZ / 6.0;
  // Correct the outputs with the calculated error values
  MagX = ( MagX - MagErrorX ) * MagScaleX;
  MagY = ( MagY - MagErrorY ) * MagScaleY;
  MagZ = ( MagZ - MagErrorZ ) * MagScaleZ;
  // LP filter magnetometer data
  MagX = ( 1.0 - config.B_mag ) * MagX_prev + config.B_mag * MagX;
  MagY = ( 1.0 - config.B_mag ) * MagY_prev + config.B_mag * MagY;
  MagZ = ( 1.0 - config.B_mag ) * MagZ_prev + config.B_mag * MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
#endif
}

void calculate_IMU_error()
{
  // DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement.
   */
#if defined USE_MPU_9DOF
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
#else
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
#endif

  // Read IMU values c = IMU_CAL_ITERATION_LOOPS times
  int c = 0;
  while ( c < IMU_CAL_ITERATION_LOOPS ) {
#if defined USE_MPU_9DOF
    mpu_read_imu9( &AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ );
#else
    mpu_read_imu( &AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ );
#endif

    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    // Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  // Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;
}

void calibrateAttitude()
{
  // DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  // Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot.
   */
  // Warm up IMU and madgwick filter in simulated main loop
  for ( int i = 0; i <= ATTITUDE_CAL_ITERATION_LOOPS; i++ ) {
    prev_time = current_time;
    current_time = micros();
    dt = ( current_time - prev_time ) / 1000000.0;
    getIMUdata();
    Madgwick( GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt );
    loopRate( 2000 ); // Not to exceed 2000Hz
  }
}

void Madgwick( float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq )
{
  // DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use 6DOF algorithm if appropriate
#if defined USE_MPU_6DOF
  Madgwick6DOF( gx, gy, gz, ax, ay, az, invSampleFreq );
  return;
#endif

  // Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ( ( mx == 0.0f ) && ( my == 0.0f ) && ( mz == 0.0f ) ) {
    Madgwick6DOF( gx, gy, gz, ax, ay, az, invSampleFreq );
    return;
  }

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ( -q1 * gx - q2 * gy - q3 * gz );
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy );
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx );
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx );

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if ( !( ( ax == 0.0f ) && ( ay == 0.0f ) && ( az == 0.0f ) ) ) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt( ax * ax + ay * ay + az * az );
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt( mx * mx + my * my + mz * mz );
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf( hx * hx + hy * hy );
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * ( 2.0f * q1q3 - _2q0q2 - ax ) + _2q1 * ( 2.0f * q0q1 + _2q2q3 - ay ) - _2bz * q2 * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - mx ) + ( -_2bx * q3 + _2bz * q1 ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - my ) + _2bx * q2 * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - mz );
    s1 = _2q3 * ( 2.0f * q1q3 - _2q0q2 - ax ) + _2q0 * ( 2.0f * q0q1 + _2q2q3 - ay ) - 4.0f * q1 * ( 1 - 2.0f * q1q1 - 2.0f * q2q2 - az ) + _2bz * q3 * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - mx ) + ( _2bx * q2 + _2bz * q0 ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - my ) + ( _2bx * q3 - _4bz * q1 ) * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - mz );
    s2 = -_2q0 * ( 2.0f * q1q3 - _2q0q2 - ax ) + _2q3 * ( 2.0f * q0q1 + _2q2q3 - ay ) - 4.0f * q2 * ( 1 - 2.0f * q1q1 - 2.0f * q2q2 - az ) + ( -_4bx * q2 - _2bz * q0 ) * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - mx ) + ( _2bx * q1 + _2bz * q3 ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - my ) + ( _2bx * q0 - _4bz * q2 ) * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - mz );
    s3 = _2q1 * ( 2.0f * q1q3 - _2q0q2 - ax ) + _2q2 * ( 2.0f * q0q1 + _2q2q3 - ay ) + ( -_4bx * q3 + _2bz * q1 ) * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - mx ) + ( -_2bx * q0 + _2bz * q2 ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - my ) + _2bx * q1 * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - mz );
    recipNorm = invSqrt( s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3 ); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= config.B_madgwick * s0;
    qDot2 -= config.B_madgwick * s1;
    qDot3 -= config.B_madgwick * s2;
    qDot4 -= config.B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  // Normalize quaternion
  recipNorm = invSqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // compute angles - NWU
  roll_IMU = atan2( q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2 ) * 57.29577951; // degrees
  pitch_IMU = -asin( -2.0f * ( q1 * q3 - q0 * q2 ) ) * 57.29577951; // degrees
  yaw_IMU = -atan2( q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3 ) * 57.29577951; // degrees
}

void Madgwick6DOF( float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq )
{
  // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ( -q1 * gx - q2 * gy - q3 * gz );
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy );
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx );
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx );

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if ( !( ( ax == 0.0f ) && ( ay == 0.0f ) && ( az == 0.0f ) ) ) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt( ax * ax + ay * ay + az * az );
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt( s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3 ); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= config.B_madgwick * s0;
    qDot2 -= config.B_madgwick * s1;
    qDot3 -= config.B_madgwick * s2;
    qDot4 -= config.B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  // Normalise quaternion
  recipNorm = invSqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Compute angles
  roll_IMU = atan2( q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2 ) * 57.29577951; // degrees
  pitch_IMU = -asin( -2.0f * ( q1 * q3 - q0 * q2 ) ) * 57.29577951; // degrees
  yaw_IMU = -atan2( q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3 ) * 57.29577951; // degrees
}

void getDesState()
{
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = ( channel_1_pwm - 1000.0 ) / 1000.0; // Between 0 and 1
  roll_des = ( channel_2_pwm - 1500.0 ) / 500.0; // Between -1 and 1
  pitch_des = ( channel_3_pwm - 1500.0 ) / 500.0; // Between -1 and 1
  yaw_des = ( channel_4_pwm - 1500.0 ) / 500.0; // Between -1 and 1
  roll_passthru = roll_des / 2.0; // Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0; // Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0; // Between -0.5 and 0.5

  // Constrain within normalized bounds
  thro_des = constrain( thro_des, 0.0, 1.0 ); // Between 0 and 1
  roll_des = constrain( roll_des, -1.0, 1.0 ) * config.maxRoll; // Between -maxRoll and +maxRoll
  pitch_des = constrain( pitch_des, -1.0, 1.0 ) * config.maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain( yaw_des, -1.0, 1.0 ) * config.maxYaw; // Between -maxYaw and +maxYaw
  roll_passthru = constrain( roll_passthru, -0.5, 0.5 );
  pitch_passthru = constrain( pitch_passthru, -0.5, 0.5 );
  yaw_passthru = constrain( yaw_passthru, -0.5, 0.5 );
}

void controlANGLE()
{
  // DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain( integral_roll, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01 * ( config.Kp_roll_angle * error_roll + config.Ki_roll_angle * integral_roll - config.Kd_roll_angle * derivative_roll ); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain( integral_pitch, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01 * ( config.Kp_pitch_angle * error_pitch + config.Ki_pitch_angle * integral_pitch - config.Kd_pitch_angle * derivative_pitch ); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain( integral_yaw, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = ( error_yaw - error_yaw_prev ) / dt;
  yaw_PID = .01 * ( config.Kp_yaw * error_yaw + config.Ki_yaw * integral_yaw + config.Kd_yaw * derivative_yaw ); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  // Update pitch variables
  integral_pitch_prev = integral_pitch;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2()
{
  // DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  // Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain( integral_roll_ol, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_roll = ( roll_IMU - roll_IMU_prev ) / dt;
  roll_des_ol = config.Kp_roll_angle * error_roll + config.Ki_roll_angle * integral_roll_ol; // - Kd_roll_angle*derivative_roll;

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain( integral_pitch_ol, -config.i_limit, config.i_limit ); // saturate integrator to prevent unsafe buildup
  derivative_pitch = ( pitch_IMU - pitch_IMU_prev ) / dt;
  pitch_des_ol = config.Kp_pitch_angle * error_pitch + config.Ki_pitch_angle * integral_pitch_ol; // - Kd_pitch_angle*derivative_pitch;

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl * roll_des_ol;
  pitch_des_ol = Kl * pitch_des_ol;
  roll_des_ol = constrain( roll_des_ol, -240.0, 240.0 );
  pitch_des_ol = constrain( pitch_des_ol, -240.0, 240.0 );
  roll_des_ol = ( 1.0 - config.B_loop_roll ) * roll_des_prev + config.B_loop_roll * roll_des_ol;
  pitch_des_ol = ( 1.0 - config.B_loop_pitch ) * pitch_des_prev + config.B_loop_pitch * pitch_des_ol;

  // Inner loop - PID on rate
  // Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain( integral_roll_il, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_roll = ( error_roll - error_roll_prev ) / dt;
  roll_PID = .01 * ( config.Kp_roll_rate * error_roll + config.Ki_roll_rate * integral_roll_il + config.Kd_roll_rate * derivative_roll ); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain( integral_pitch_il, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = ( error_pitch - error_pitch_prev ) / dt;
  pitch_PID = .01 * ( config.Kp_pitch_rate * error_pitch + config.Ki_pitch_rate * integral_pitch_il + config.Kd_pitch_rate * derivative_pitch ); // Scaled by .01 to bring within -1 to 1 range

  // Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain( integral_yaw, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = ( error_yaw - error_yaw_prev ) / dt;
  yaw_PID = .01 * ( config.Kp_yaw * error_yaw + config.Ki_yaw * integral_yaw + config.Kd_yaw * derivative_yaw ); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  // Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE()
{
  // DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  // Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain( integral_roll, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_roll = ( error_roll - error_roll_prev ) / dt;
  roll_PID = .01 * ( config.Kp_roll_rate * error_roll + config.Ki_roll_rate * integral_roll + config.Kd_roll_rate * derivative_roll ); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain( integral_pitch, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = ( error_pitch - error_pitch_prev ) / dt;
  pitch_PID = .01 * ( config.Kp_pitch_rate * error_pitch + config.Ki_pitch_rate * integral_pitch + config.Kd_pitch_rate * derivative_pitch ); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if ( channel_1_pwm < 1060 ) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain( integral_yaw, -config.i_limit, config.i_limit ); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = ( error_yaw - error_yaw_prev ) / dt;
  yaw_PID = .01 * ( config.Kp_yaw * error_yaw + config.Ki_yaw * integral_yaw + config.Kd_yaw * derivative_yaw ); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  // Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands()
{
  // DESCRIPTION: Scale and limit normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_summed variables from the mixer function are scaled to 0-1 for motor commands. sX_command_summed variables from
   * the mixer function are scaled to 0-1 for the servo library using standard PWM.
   * mX_command are updated here which are used to command the motors in commandMotors(). sX_command are also updated
   * which are used to command the servos.
   */
  // Constrain commands to motors within oneshot125 bounds
  m1_command = constrain( m1_command_summed, 0.0, 1.0 );
  m2_command = constrain( m2_command_summed, 0.0, 1.0 );
  m3_command = constrain( m3_command_summed, 0.0, 1.0 );
  m4_command = constrain( m4_command_summed, 0.0, 1.0 );
  m5_command = constrain( m5_command_summed, 0.0, 1.0 );
  m6_command = constrain( m6_command_summed, 0.0, 1.0 );

  // Constrain commands to servos within servo library bounds
  s1_command = constrain( s1_command_summed, 0.0, 1.0 );
  s2_command = constrain( s2_command_summed, 0.0, 1.0 );
  s3_command = constrain( s3_command_summed, 0.0, 1.0 );
  s4_command = constrain( s4_command_summed, 0.0, 1.0 );
  s5_command = constrain( s5_command_summed, 0.0, 1.0 );
  s6_command = constrain( s6_command_summed, 0.0, 1.0 );
  s7_command = constrain( s7_command_summed, 0.0, 1.0 );
}

void getCommands()
{
  // DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
   */

// #if defined USE_PPM_RX || defined USE_PWM_RX
//   channel_1_pwm = getRadioPWM( 1 );
//   channel_2_pwm = getRadioPWM( 2 );
//   channel_3_pwm = getRadioPWM( 3 );
//   channel_4_pwm = getRadioPWM( 4 );
//   channel_5_pwm = getRadioPWM( 5 );
//   channel_6_pwm = getRadioPWM( 6 );

// #elif defined USE_SBUS_RX
//   if ( sbus.read( &sbusChannels[0], &sbusFailSafe, &sbusLostFrame ) ) {
//     //sBus scaling below is for Taranis-Plus and X4R-SB
//     float scale = 0.615;
//     float bias  = 895.0;
//     channel_1_pwm = sbusChannels[0] * scale + bias;
//     channel_2_pwm = sbusChannels[1] * scale + bias;
//     channel_3_pwm = sbusChannels[2] * scale + bias;
//     channel_4_pwm = sbusChannels[3] * scale + bias;
//     channel_5_pwm = sbusChannels[4] * scale + bias;
//     channel_6_pwm = sbusChannels[5] * scale + bias;
//   }
// #endif

  // rc_update( rc_command_uSec, COUNT_OF( rc_command_uSec ) );
  UW_rx_update( rc_command_uSec, NELEMENTS( rc_command_uSec ), RCVR_PROTOCOL );

  channel_1_pwm = rc_command_uSec[ 2 ]; // thro
  channel_2_pwm = rc_command_uSec[ 0 ]; // ail
  channel_3_pwm = rc_command_uSec[ 1 ]; // elev
  channel_4_pwm = rc_command_uSec[ 3 ]; // rudd
  channel_5_pwm = rc_command_uSec[ 6 ]; // throttle cut (SWA)
  channel_6_pwm = rc_command_uSec[ 9 ]; // aux1 (SWD)

  // Low-pass the critical commands and update previous values
  float b = 0.2; // lower=slower, higher=noiser
  channel_1_pwm = ( 1.0 - b ) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = ( 1.0 - b ) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = ( 1.0 - b ) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = ( 1.0 - b ) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe()
{
  // DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  // Triggers for failure criteria
  if ( channel_1_pwm > maxVal || channel_1_pwm < minVal ) check1 = 1;
  if ( channel_2_pwm > maxVal || channel_2_pwm < minVal ) check2 = 1;
  if ( channel_3_pwm > maxVal || channel_3_pwm < minVal ) check3 = 1;
  if ( channel_4_pwm > maxVal || channel_4_pwm < minVal ) check4 = 1;
  if ( channel_5_pwm > maxVal || channel_5_pwm < minVal ) check5 = 1;
  if ( channel_6_pwm > maxVal || channel_6_pwm < minVal ) check6 = 1;

  // If any failures, set to default failsafe values
  if ( ( check1 + check2 + check3 + check4 + check5 + check6 ) > 0 ) {
    channel_1_pwm = config.channel_1_fs;
    channel_2_pwm = config.channel_2_fs;
    channel_3_pwm = config.channel_3_fs;
    channel_4_pwm = config.channel_4_fs;
    channel_5_pwm = config.channel_5_fs;
    channel_6_pwm = config.channel_6_fs;
  }
}

void commandMotors()
{
  // DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;

#if 0 // GLS

  // Write all motor pins high
  digitalWrite( m1Pin, HIGH );
  digitalWrite( m2Pin, HIGH );
  digitalWrite( m3Pin, HIGH );
  digitalWrite( m4Pin, HIGH );
  digitalWrite( m5Pin, HIGH );
  digitalWrite( m6Pin, HIGH );
  pulseStart = micros();

  // Write each motor pin low as correct pulse length is reached
  while ( wentLow < 6 ) { // Keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ( ( m1_command_PWM <= timer - pulseStart ) && ( flagM1 == 0 ) ) {
      digitalWrite( m1Pin, LOW );
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ( ( m2_command_PWM <= timer - pulseStart ) && ( flagM2 == 0 ) ) {
      digitalWrite( m2Pin, LOW );
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ( ( m3_command_PWM <= timer - pulseStart ) && ( flagM3 == 0 ) ) {
      digitalWrite( m3Pin, LOW );
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ( ( m4_command_PWM <= timer - pulseStart ) && ( flagM4 == 0 ) ) {
      digitalWrite( m4Pin, LOW );
      wentLow = wentLow + 1;
      flagM4 = 1;
    }
    if ( ( m5_command_PWM <= timer - pulseStart ) && ( flagM5 == 0 ) ) {
      digitalWrite( m5Pin, LOW );
      wentLow = wentLow + 1;
      flagM5 = 1;
    }
    if ( ( m6_command_PWM <= timer - pulseStart ) && ( flagM6 == 0 ) ) {
      digitalWrite( m6Pin, LOW );
      wentLow = wentLow + 1;
      flagM6 = 1;
    }
  }

#endif // GLS

}

void armMotors()
{
  // DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for ( int i = 0; i <= 50; i++ ) {
    commandMotors();
    UW_time_delay_ms( 2 );
  }
}

void calibrateESCs()
{
  // DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
  while ( true ) {
    prev_time = current_time;
    current_time = micros();
    dt = ( current_time - prev_time ) / 1000000.0;

    UW_led_heartbeat_on(); // LED on to indicate we are not in main loop

    getCommands(); // Pulls current available radio commands
    failSafe(); // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
    getDesState(); // Convert raw commands to normalized values based on saturated control limits
    getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick( GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt ); // Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
    getDesState(); // Convert raw commands to normalized values based on saturated control limits

    m1_command_summed = thro_des;
    m2_command_summed = thro_des;
    m3_command_summed = thro_des;
    m4_command_summed = thro_des;
    m5_command_summed = thro_des;
    m6_command_summed = thro_des;
    s1_command_summed = thro_des;
    s2_command_summed = thro_des;
    s3_command_summed = thro_des;
    s4_command_summed = thro_des;
    s5_command_summed = thro_des;
    s6_command_summed = thro_des;
    s7_command_summed = thro_des;
    scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

    // throttleCut(); // Directly sets motor commands to low based on state of ch5

    // servo1.write( s1_command_PWM );
    // servo2.write( s2_command_PWM );
    // servo3.write( s3_command_PWM );
    // servo4.write( s4_command_PWM );
    // servo5.write( s5_command_PWM );
    // servo6.write( s6_command_PWM );
    // servo7.write( s7_command_PWM );
    commandMotors(); // Sends command pulses to each motor pin using OneShot125 protocol

    // printRadioData(); // Radio pwm values (expected: 1000 to 2000)

    loopRate( 2000 ); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
  }
}

float floatFaderLinear( float param, float param_min, float param_max, float fadeTime, int state, int loopFreq )
{
  // DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *
   */
  float diffParam = ( param_max - param_min ) / ( fadeTime * loopFreq ); // Difference to add or subtract from param for each loop iteration for desired fadeTime

  if ( state == 1 ) { // Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  } else if ( state == 0 ) { // Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain( param, param_min, param_max ); // Constrain param within max bounds

  return param;
}

float floatFaderLinear2( float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq )
{
  // DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
   *  statements in order to fade controller gains, for example between the two dynamic configurations.
   *
   */
  if ( param > param_des ) { // Need to fade down to get to desired
    float diffParam = ( param_upper - param_des ) / ( fadeTime_down * loopFreq );
    param = param - diffParam;
  } else if ( param < param_des ) { // Need to fade up to get to desired
    float diffParam = ( param_des - param_lower ) / ( fadeTime_up * loopFreq );
    param = param + diffParam;
  }

  param = constrain( param, param_lower, param_upper ); // Constrain param within max bounds

  return param;
}

void switchRollYaw( int reverseRoll, int reverseYaw )
{
  // DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not
   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis.
   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the
   * IMU tilted 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

void throttleCut()
{
  // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
   * minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first.
   */
  if ( channel_5_pwm > 1500 ) {
    m1_command = -.05;
    m2_command = -.05;
    m3_command = -.05;
    m4_command = -.05;
    m5_command = -.05;
    m6_command = -.05;

    // Uncomment if using servo PWM variables to control motor ESCs
    // s1_command_PWM = 0;
    // s2_command_PWM = 0;
    // s3_command_PWM = 0;
    // s4_command_PWM = 0;
    // s5_command_PWM = 0;
    // s6_command_PWM = 0;
    // s7_command_PWM = 0;
  }
}

void loopRate( int freq )
{
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while ( invFreq > ( checker - current_time ) ) {
    checker = micros();
  }
}

void loopBlink()
{
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  if ( current_time - blink_counter > blink_delay ) {
    blink_counter = micros();
    UW_led_heartbeat_toggle();

    if ( blinkAlternate == 1 ) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if ( blinkAlternate == 0 ) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink( int numBlinks, int upTime, int downTime )
{
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for ( int j = 1; j <= numBlinks; j++ ) {
    UW_led_heartbeat_on();
    UW_time_delay_ms( downTime );
    UW_led_heartbeat_off();
    UW_time_delay_ms( upTime );
  }
}

//=========================================================================================//

// HELPER FUNCTIONS

float invSqrt( float x )
{
  // Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  // Alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0 / sqrtf( x );
}

//========================================================================================================================//
//                                                 DEBUG PRINT FUNCTIONS                                                  //
//========================================================================================================================//

#define PRN_BUFFER_SIZE  128
static char prn_buffer[PRN_BUFFER_SIZE] = {0};

void printRadioData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "CH1: %-6d CH2: %-6d CH3: %-6d CH4: %-6d CH5: %-6d CH6: %-6d\r\n", \
          ( int )channel_1_pwm, \
          ( int )channel_2_pwm, \
          ( int )channel_3_pwm, \
          ( int )channel_4_pwm, \
          ( int )channel_5_pwm, \
          ( int )channel_6_pwm \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printDesiredState()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "thro_des: %5.3f roll_des: %5.3f pitch_des: %5.3f yaw_des: %7.2f\r\n", \
          thro_des, \
          roll_des, \
          pitch_des, \
          yaw_des \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printGyroData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "GyroX: %7.1f GyroY: %7.1f GyroZ: %7.1f\r\n", \
          GyroX, \
          GyroY, \
          GyroZ \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printAccelData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "AccX: %7.3f AccY: %7.3f AccZ: %7.3f\r\n", \
          AccX, \
          AccY, \
          AccZ \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printMagData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "MagX: %7.1f MagY: %7.1f MagZ: %7.1f\r\n", \
          MagX, \
          MagY, \
          MagZ \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printRollPitchYaw()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "roll: %7.3f pitch: %7.3f yaw: %7.3f\r\n", \
          roll_IMU, \
          pitch_IMU, \
          yaw_IMU \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printPIDoutput()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "roll_PID: %6.3f  pitch_PID: %6.3f  yaw_PID: %6.3f\r\n", \
          roll_PID, \
          pitch_PID, \
          yaw_PID \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printMotorCommands()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "m1_command: %6.3f m2_command: %6.3f m3_command: %6.3f m4_command: %6.3f m5_command: %6.3f m6_command: %6.3f\r\n", \
          m1_command, \
          m2_command, \
          m3_command, \
          m4_command, \
          m5_command, \
          m6_command \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printServoCommands()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "s1_cmd: %5d s2_cmd: %5d s3_cmd: %5d s4_cmd: %5d s5_cmd: %5d s6_cmd: %5d s7_cmd: %5d\r\n", \
          ( int )s1_command, \
          ( int )s2_command, \
          ( int )s3_command, \
          ( int )s4_command, \
          ( int )s5_command, \
          ( int )s6_command, \
          ( int )s7_command \
        );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printLoopRate()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("dt = "));
    // Serial.println(dt*1000000.0);
    uint16_t prn_buflen = snprintf_( prn_buffer, sizeof( prn_buffer ), "dt(sec.) = %10.5f\r\n", dt  );
    UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  }
}