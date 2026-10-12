#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

// Sensitivity at the configured full-scale ranges (see gyro_init):
// gyro  +-250 deg/s  -> 131 LSB per deg/s
// accel +-2 g        -> 16384 LSB per g
#define MPU_GYRO_SCALE 131
#define MPU_ACCEL_SCALE 16384

typedef struct {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
} mpu_data_t;

uint8_t gyro_init(void);
void gyro_read(mpu_data_t *d);
void gyro_read_gyro(mpu_data_t *d);
void gyro_read_gy(mpu_data_t *d);
void gyro_read_accel(mpu_data_t *d);

#endif
