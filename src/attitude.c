#include "attitude.h"
#include "gyro.h"

#define ALPHA 248
#define GYRO_SCALE 131

static int32_t angle_pitch;
static int32_t angle_roll;

static int32_t iatan2(int32_t y, int32_t x) {
  int32_t ax = x >= 0 ? x : -x;
  int32_t ay = y >= 0 ? y : -y;
  int32_t d;
  if (ax >= ay)
    d = ay * 4500 / (ax + 1);
  else
    d = 9000 - ax * 4500 / (ay + 1);
  if (x < 0)
    d = 18000 - d;
  if (y < 0)
    d = -d;
  return d;
}

void attitude_init(void) {
  mpu_data_t d;
  int32_t sum_p = 0, sum_r = 0;

  for (uint8_t i = 0; i < 64; i++) {
    gyro_read(&d);
    sum_p += iatan2(-(int32_t)d.ax, (int32_t)d.az);
    sum_r += iatan2((int32_t)d.ay, (int32_t)d.az);
  }

  angle_pitch = sum_p / 64;
  angle_roll = sum_r / 64;
}

void attitude_update(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
                     int16_t gz) {
  (void)gz;
  angle_pitch += gx / GYRO_SCALE;
  angle_roll += gy / GYRO_SCALE;

  int32_t a_pitch = iatan2(-(int32_t)ax, (int32_t)az);
  int32_t a_roll = iatan2((int32_t)ay, (int32_t)az);

  angle_pitch = (ALPHA * angle_pitch + (256 - ALPHA) * a_pitch) >> 8;
  angle_roll = (ALPHA * angle_roll + (256 - ALPHA) * a_roll) >> 8;
}

int16_t attitude_pitch(void) { return (int16_t)(angle_pitch / 10); }
int16_t attitude_roll(void) { return (int16_t)(angle_roll / 10); }
