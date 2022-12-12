#include <uvos.h>

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param  ax 16-bit signed integer for accelerometer X-axis value
 *         ay 16-bit signed integer for accelerometer Y-axis value
 *         az 16-bit signed integer for accelerometer Z-axis value
 *         gx 16-bit signed integer for gyroscope X-axis value
 *         gy 16-bit signed integer for gyroscope Y-axis value
 *         gz 16-bit signed integer for gyroscope Z-axis value
 * @retval None
 */
uint32_t mpu_read_imu( int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz )
{
  return UVOS_MPU_ReadImu( ax, ay,  az, gx, gy, gz );
}

uint32_t mpu_set_imu_ranges( enum uvos_mpu_gyro_range gyroRange, enum uvos_mpu_accel_range accelRange, enum uvos_mpu_filter filterSetting )
{
  return UVOS_MPU_ConfigureRanges( gyroRange, accelRange, filterSetting );
}

void mpu_set_speed_slow( void )
{
  UVOS_MPU_SetSpeedFast( false );
}

void mpu_set_speed_fast( void )
{
  UVOS_MPU_SetSpeedFast( true );
}