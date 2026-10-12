#include "gyro.h"
#include "twi.h"

#define MPU_ADDR 0x68
#define MPU_WR (MPU_ADDR << 1)
#define MPU_RD (MPU_ADDR << 1 | 1)

#define REG_PWR       0x6B
#define REG_CONFIG    0x1A
#define REG_GYRO_CFG  0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_ACCEL     0x3B
#define REG_GYRO      0x43
#define REG_WHOAMI    0x75

// Digital low-pass filter (CONFIG.DLPF_CFG): 3 => gyro ~42 Hz / accel ~44 Hz
// bandwidth. Keeps vibration above ~40 Hz from aliasing into the 100 Hz loop.
#define DLPF_42HZ 0x03

// Gyro full scale +-250 deg/s (FS_SEL=0, MPU_GYRO_SCALE), accel +-2 g (AFS_SEL=0).
#define GYRO_FS_250  0x00
#define ACCEL_FS_2G  0x00

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
        // Wake the MPU6050 and clock it from the gyro X PLL (more stable than
        // the internal 8 MHz oscillator).
        if (write_reg(REG_PWR, 0x01))
            continue;
        // Enable the digital low-pass filter to reject high-frequency noise.
        if (write_reg(REG_CONFIG, DLPF_42HZ))
            continue;
        // Lock the full-scale ranges so the scale constants are deterministic.
        if (write_reg(REG_GYRO_CFG, GYRO_FS_250))
            continue;
        if (write_reg(REG_ACCEL_CFG, ACCEL_FS_2G))
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
