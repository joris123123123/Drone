#include "gyro.h"
#include "twi.h"

#define MPU_ADDR 0x68
#define MPU_WR (MPU_ADDR << 1)
#define MPU_RD (MPU_ADDR << 1 | 1)

#define REG_PWR   0x6B
#define REG_ACCEL 0x3B
#define REG_GYRO  0x43
#define REG_WHOAMI 0x75

#define GYRO_RETRIES 3

static uint8_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t ok;
    ok = twi_start();
    if (ok) return 1;
    ok = twi_write(MPU_WR);
    if (ok) return 1;
    ok = twi_write(reg);
    if (ok) return 1;
    ok = twi_write(val);
    if (ok) return 1;
    twi_stop();
    return 0;
}

static uint8_t read_buf(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t ok;
    ok = twi_start();
    if (ok) return 1;
    ok = twi_write(MPU_WR);
    if (ok) return 1;
    ok = twi_write(reg);
    if (ok) return 1;
    ok = twi_start();
    if (ok) return 1;
    ok = twi_write(MPU_RD);
    if (ok) return 1;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t last = (i == len - 1);
        if (twi_read(!last, &buf[i]))
            return 1;
    }
    twi_stop();
    return 0;
}

uint8_t gyro_init(void) {
    for (uint8_t retry = 0; retry < GYRO_RETRIES; retry++) {
        if (write_reg(REG_PWR, 0x00))
            continue;
        uint8_t id = 0;
        if (read_buf(REG_WHOAMI, &id, 1))
            continue;
        if (id == 0x68)
            return 1;
    }
    return 0;
}

void gyro_read(mpu_data_t *d) {
    uint8_t buf[14];
    if (read_buf(REG_ACCEL, buf, 14)) {
        d->ax = 0;
        d->ay = 0;
        d->az = 0;
        d->gx = 0;
        d->gy = 0;
        d->gz = 0;
        d->temp = 0;
        return;
    }
    d->ax   = (int16_t)(buf[0]  << 8 | buf[1]);
    d->ay   = (int16_t)(buf[2]  << 8 | buf[3]);
    d->az   = (int16_t)(buf[4]  << 8 | buf[5]);
    d->temp = (int16_t)(buf[6]  << 8 | buf[7]);
    d->gx   = (int16_t)(buf[8]  << 8 | buf[9]);
    d->gy   = (int16_t)(buf[10] << 8 | buf[11]);
    d->gz   = (int16_t)(buf[12] << 8 | buf[13]);
}

void gyro_read_gyro(mpu_data_t *d) {
    uint8_t buf[6];
    if (read_buf(REG_GYRO, buf, 6)) {
        d->gx = 0; d->gy = 0; d->gz = 0;
        return;
    }
    d->gx = (int16_t)(buf[0] << 8 | buf[1]);
    d->gy = (int16_t)(buf[2] << 8 | buf[3]);
    d->gz = (int16_t)(buf[4] << 8 | buf[5]);
}

void gyro_read_gy(mpu_data_t *d) {
    uint8_t buf[2];
    if (read_buf(REG_GYRO + 2, buf, 2)) {
        d->gy = 0;
        return;
    }
    d->gy = (int16_t)(buf[0] << 8 | buf[1]);
}

void gyro_read_accel(mpu_data_t *d) {
    uint8_t buf[6];
    if (read_buf(REG_ACCEL, buf, 6)) {
        d->ax = 0; d->ay = 0; d->az = 0;
        return;
    }
    d->ax = (int16_t)(buf[0] << 8 | buf[1]);
    d->ay = (int16_t)(buf[2] << 8 | buf[3]);
    d->az = (int16_t)(buf[4] << 8 | buf[5]);
}
