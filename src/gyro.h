#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

typedef struct {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
} mpu_data_t;

uint8_t gyro_init(void);
void gyro_read(mpu_data_t *d);

#endif
