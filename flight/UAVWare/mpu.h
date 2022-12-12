#ifndef MPU_H
#define MPU_H

#ifdef __cplusplus
extern "C" {
#endif

// bool mpu_init( void );
extern void mpu_read_imu( int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz );
extern uint32_t mpu_set_imu_ranges( enum uvos_mpu_gyro_range gyroRange, enum uvos_mpu_accel_range accelRange, enum uvos_mpu_filter filterSetting );
extern void mpu_set_speed_slow( void );
extern void mpu_set_speed_fast( void );

#ifdef __cplusplus
}
#endif

#endif // MPU_H