// src/gyro.c

#include "gyro.h"

#define MPU_ADDR 0x68
#define MPU_WR (MPU_ADDR << 1)
#define MPU_RD (MPU_ADDR << 1 | 1)

#define REG_PWR 0x6B
#define REG_ACCEL 0x3B

static void write_reg(uint8_t reg, uint8_t val) {
  twi_start();
  twi_write(MPU_WR);
  twi_write(reg);
  twi_write(val);
  twi_stop();
}

static void read_buf(uint8_t reg, uint8_t *buf, uint8_t len) {
  twi_start();
  twi_write(MPU_WR);
  twi_write(reg);
  twi_start();
  twi_write(MPU_RD);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = twi_read(i < len - 1);
  twi_stop();
}

uint8_t gyro_init(void) {
  write_reg(REG_PWR, 0x00);
  uint8_t id = 0;
  read_buf(0x75, &id, 1);
  return id == 0x68;
}

void gyro_read(mpu_data_t *d) {
  uint8_t buf[14];
  read_buf(REG_ACCEL, buf, 14);
  d->ax = (int16_t)(buf[0] << 8 | buf[1]);
  d->ay = (int16_t)(buf[2] << 8 | buf[3]);
  d->az = (int16_t)(buf[4] << 8 | buf[5]);
  d->temp = (int16_t)(buf[6] << 8 | buf[7]);
  d->gx = (int16_t)(buf[8] << 8 | buf[9]);
  d->gy = (int16_t)(buf[10] << 8 | buf[11]);
  d->gz = (int16_t)(buf[12] << 8 | buf[13]);
}
