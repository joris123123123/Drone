#include "attitude.h"
#include "gyro.h"

/*
 * Complementary attitude filter (Mahony style) estimating pitch and roll from
 * the MPU6050 gyroscope + accelerometer.
 *
 * The gyro is integrated for fast response, the accelerometer provides a
 * long-term gravity reference that continuously corrects gyro drift, and a
 * slow integral term estimates the residual gyro bias left after the boot
 * calibration:
 *
 *   gyro'  = gyro - bias
 *   angle += gyro' * dt                             (integrate, fast)
 *   angle += ACCEL_ALPHA/256 * (accel_angle - angle) (accel drift correction)
 *   bias  -= BIAS_KI/256     * (accel_angle - angle) (slow bias trim)
 *
 * The bias sign matters: if the gyro reads high, the integrated angle runs
 * ahead of the accelerometer angle (err < 0) and the bias estimate must grow
 * so that (gyro - bias) shrinks. With the opposite sign the loop is unstable.
 *
 * The accelerometer is only trusted while its magnitude is close to 1 g, so
 * linear acceleration/thrust does not corrupt the angle estimate.
 *
 * Everything is integer fixed point:
 *   angle       : Q8 centidegrees (0.01 deg, stored *256)
 *   gyro / bias : raw LSB (MPU_GYRO_SCALE LSB per deg/s)
 *   accel_angle : centidegrees (0.01 deg)
 *
 * The control loop runs at exactly 100 Hz (CTRL_RATE_HZ in main.c); the
 * per-sample gyro integration constant below folds in dt = 10 ms:
 *   deg/s = lsb / 131
 *   centideg increment = deg/s * dt * 100 = (lsb / 131) * 0.01 * 100 = lsb / 131
 * If the control rate changes this constant must be rescaled.
 */

// Accelerometer drift-correction strength: ACCEL_ALPHA/256 per sample.
// 8/256 at 100 Hz -> ~0.32 s time constant.
#define ACCEL_ALPHA 8

// Residual gyro-bias trim: BIAS_KI/256 per sample (~11 s convergence,
// monotonically damped). Kept slow so accelerometer noise averages out.
#define BIAS_KI 4
#define BIAS_LIMIT_Q8 (2000L * 256) // +-15 deg/s in Q8 LSB

// Accept the accelerometer as a gravity reference only when its magnitude is
// within 0.75..1.25 g. Values are (vector magnitude)^2 with every raw axis
// pre-shifted right by 2 to keep the sum of squares inside int32.
#define ACCEL_MIN2 (3072L * 3072L) // (0.75 g)^2
#define ACCEL_MAX2 (5120L * 5120L) // (1.25 g)^2

static int32_t angle_pitch_q8;
static int32_t angle_roll_q8;
static int32_t bias_pitch_q8;
static int32_t bias_roll_q8;

// Bias-corrected body rates in raw gyro LSB, published for the rate controller
// (see attitude_rate_pitch/roll). They stay 0 until the first update.
static int16_t rate_pitch_lsb;
static int16_t rate_roll_lsb;

// Accelerometer-angle reference captured at power-up. Subtracting it makes the
// orientation the board is in when the firmware starts read as 0 deg, so a
// fixed accel offset / mounting tilt does not show up as a constant bias.
static int32_t a_pitch_off_cd;
static int32_t a_roll_off_cd;

/*
 * atan2 approximation returning centidegrees (-18000..18000).
 * Uses atan(z) ~= z / (1 + 0.28 z^2) evaluated in Q8 fixed point, which is
 * accurate to ~0.34 deg over the whole range (the previous linear
 * approximation was off by up to ~4 deg).
 */
static int32_t iatan2(int32_t y, int32_t x) {
  int32_t ax = x >= 0 ? x : -x;
  int32_t ay = y >= 0 ? y : -y;
  int32_t d;

  if (ax == 0 && ay == 0)
    return 0;

  if (ax >= ay) {
    int32_t z = ax ? (ay * 256) / ax : 0; // Q8 ratio in 0..256
    d = (int32_t)(1466772L * z) / (65536 + (28 * z * z) / 100);
  } else {
    int32_t z = (ax * 256) / ay; // Q8 ratio in 0..256
    d = 9000 - (int32_t)(1466772L * z) / (65536 + (28 * z * z) / 100);
  }

  if (x < 0)
    d = 18000 - d;
  if (y < 0)
    d = -d;
  return d;
}

/*
 * One filter axis: integrate the (bias-corrected) gyro and blend in the
 * accelerometer angle, then adapt the gyro-bias estimate when the accel is
 * trustworthy.
 */
static int16_t filter_axis(int32_t *angle_q8, int32_t *bias_q8,
                           int32_t gyro_lsb, int32_t accel_cd,
                           uint8_t accel_ok) {
  int32_t gcorr_q8 = (gyro_lsb << 8) - *bias_q8;
  *angle_q8 += gcorr_q8 / MPU_GYRO_SCALE;

  int32_t err_q8 = (accel_cd << 8) - *angle_q8;
  *angle_q8 += (err_q8 * ACCEL_ALPHA) >> 8;

  if (accel_ok) {
    *bias_q8 -= (err_q8 * BIAS_KI) >> 8;
    if (*bias_q8 > BIAS_LIMIT_Q8)
      *bias_q8 = BIAS_LIMIT_Q8;
    else if (*bias_q8 < -BIAS_LIMIT_Q8)
      *bias_q8 = -BIAS_LIMIT_Q8;
  }

  // Bias-corrected rate in raw LSB, handed to the rate controller. The raw
  // gyro drifts with temperature after the one-shot boot calibration; a
  // drifting rate bias is a permanent PID error that slowly pegs the motors
  // to one side regardless of the stick, so the controller must use this
  // tracked value instead of the raw gyro.
  return (int16_t)(gcorr_q8 >> 8);
}

void attitude_init(void) {
  mpu_data_t d;
  int32_t sum_p = 0, sum_r = 0;

  for (uint8_t i = 0; i < 64; i++) {
    gyro_read(&d);
    sum_p += iatan2(-(int32_t)d.ax, (int32_t)d.az);
    sum_r += iatan2((int32_t)d.ay, (int32_t)d.az);
  }

  angle_pitch_q8 = 0;
  angle_roll_q8 = 0;
  bias_pitch_q8 = 0;
  bias_roll_q8 = 0;

  // Use the power-up orientation as the 0 deg reference.
  a_pitch_off_cd = sum_p / 64;
  a_roll_off_cd = sum_r / 64;
}

void attitude_update(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
                     int16_t gz) {
  (void)gz;

  int32_t axs = ax >> 2, ays = ay >> 2, azs = az >> 2;
  int32_t mag2 = axs * axs + ays * ays + azs * azs;
  uint8_t accel_ok = (mag2 >= ACCEL_MIN2 && mag2 <= ACCEL_MAX2);

  int32_t a_pitch = iatan2(-(int32_t)ax, (int32_t)az) - a_pitch_off_cd;
  int32_t a_roll = iatan2((int32_t)ay, (int32_t)az) - a_roll_off_cd;

  // Pitch comes from the gravity projection in the X-Z plane, i.e. it is the
  // rotation about the Y axis -> gyro Y. Roll is in the Y-Z plane -> gyro X.
  // (control.c uses the same convention: pitch rate = gy, roll rate = gx.)
  rate_pitch_lsb =
      filter_axis(&angle_pitch_q8, &bias_pitch_q8, gy, a_pitch, accel_ok);
  rate_roll_lsb =
      filter_axis(&angle_roll_q8, &bias_roll_q8, gx, a_roll, accel_ok);
}

// angle is Q8 centidegrees. Output is in tenths of a degree (monitor.py
// divides by 10), so divide by 100 cd/deg * 256 / 10 = 2560.
int16_t attitude_pitch(void) { return (int16_t)(angle_pitch_q8 / 2560); }
int16_t attitude_roll(void) { return (int16_t)(angle_roll_q8 / 2560); }

// Bias-corrected body rates in raw gyro LSB, for the rate controller.
int16_t attitude_rate_pitch(void) { return rate_pitch_lsb; }
int16_t attitude_rate_roll(void) { return rate_roll_lsb; }
