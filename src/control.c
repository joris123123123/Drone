#include "control.h"
#include "input.h"
#include <avr/io.h>
#include <stddef.h>

#include "pid.h"

#define MOTOR_MIN 1040
#define MOTOR_MAX 2000

// Seesaw / bench tuning: only the pitch axis is driven, roll and yaw are kept
// completely out of the motor mix. Set to 0 once those axes are ready.
#define TEST_PITCH_ONLY 1

// Full-stick rate command scale. The RC channel is +-500 around centre, so the
// commanded rate in gyro LSB is 500 * RATE_SCALE; in deg/s it is
// 500 * RATE_SCALE / 131. 50 -> ~190 deg/s max, a sane 5" starting point.
// Keep it in a range that cannot overflow the 32-bit PID error.
#define RATE_SCALE 50

// Motor output smoothing. Only the *stabilisation* part of the mix is filtered
// (the pilot throttle stays crisp):
//   MOTOR_LPF  : first-order coefficient, /256 per 100 Hz loop (128 -> ~14 ms)
//   MOTOR_SLEW : max change per loop in us (caps D-term / gyro-noise spikes;
//                raise it if the stabilisation response feels weak)
#define MOTOR_LPF 128
#define MOTOR_SLEW 100

static pid_axis_t pid_roll, pid_pitch, pid_yaw;

static int16_t motor_br, motor_fl, motor_fr, motor_bl;
static int16_t throttle_base;

// Smoothed mixer deltas, indexed fr, fl, br, bl (same order as below).
static int16_t mix_filt[4];
static uint8_t mix_primed;

int32_t ctrl_pitch_err = 0;
int16_t ctrl_pitch_out = 0;
int16_t ctrl_pitch_p = 0;
int32_t ctrl_pitch_i = 0;
int16_t ctrl_pitch_d = 0;

static int16_t clamp_motor(int16_t v) {
  if (v < MOTOR_MIN)
    return MOTOR_MIN;
  if (v > MOTOR_MAX)
    return MOTOR_MAX;
  return v;
}

void control_init(void) {
  // 5" rate-mode starting point. Tune on the bench: raise Kp until it just
  // starts to oscillate then back off ~30%, add Kd to damp, then add Ki until
  // slow drift is gone without making it sluggish.
  pid_roll.kp = 0;
  pid_roll.ki = 0;
  pid_roll.kd = 0;
  pid_reset(&pid_roll);

  pid_pitch.kp = 10;
  pid_pitch.ki = 0;
  pid_pitch.kd = 0;
  pid_reset(&pid_pitch);

  pid_yaw.kp = 0;
  pid_yaw.ki = 0;
  pid_yaw.kd = 0;
  pid_reset(&pid_yaw);
}

void control_reset(void) {
  pid_reset(&pid_roll);
  pid_reset(&pid_pitch);
  pid_reset(&pid_yaw);
  mix_primed = 0; // re-prime smoothing so it never ramps from stale values
}

// First-order low-pass + slew-rate limit on one mixer delta.
static int16_t smooth_delta(uint8_t i, int16_t target) {
  if (!mix_primed) {
    mix_filt[i] = target;
    return target;
  }

  int32_t step = ((int32_t)(target - mix_filt[i]) * MOTOR_LPF) >> 8;
  if (step > MOTOR_SLEW)
    step = MOTOR_SLEW;
  else if (step < -MOTOR_SLEW)
    step = -MOTOR_SLEW;

  mix_filt[i] = (int16_t)(mix_filt[i] + step);
  return mix_filt[i];
}

void control_loop(const uint16_t *rc, int16_t gx, int16_t gy, int16_t gz) {
  throttle_base = rc[CH_THR];

  // Measured body rates, expressed in the sign convention the mixer drives.
  // A positive mixer output spins the back / left motors faster, producing a
  // nose-down / right-roll rotation, and the IMU reports those as +gy / +gx.
  // The PID needs error = setpoint - measured_rate, so the rate fed to it is
  // the *negated* raw gyro. Feeding the raw gyro straight in (as before) made
  // every axis positive feedback: any disturbance ran away to full motor
  // deflection (e.g. the back motors slamming to max the instant pitch was
  // disturbed), regardless of which way the stick was pushed.
  int16_t rate_pitch = -gy;
  int16_t rate_roll = -gx;
  int16_t rate_yaw = -gz;

  int32_t err_pitch = (int32_t)(rc[CH_PITCH] - 1500) * RATE_SCALE + rate_pitch;
  int16_t p_pitch = pid_update(&pid_pitch, err_pitch, rate_pitch, &ctrl_pitch_p,
                               &ctrl_pitch_i, &ctrl_pitch_d);

  ctrl_pitch_err = err_pitch;
  ctrl_pitch_out = p_pitch;

#if TEST_PITCH_ONLY
  // Roll and yaw idle: no RC input is read and no PID is run for them, so they
  // cannot influence the motors even if their gains are changed by accident.
  (void)rate_roll;
  (void)rate_yaw;
  int16_t p_roll = 0;
  int16_t p_yaw = 0;
#else
  int32_t err_roll = (int32_t)(rc[CH_ROLL] - 1500) * RATE_SCALE + rate_roll;
  int32_t err_yaw = (int32_t)(rc[CH_YAW] - 1500) * RATE_SCALE + rate_yaw;

  int16_t p_roll = pid_update(&pid_roll, err_roll, rate_roll, NULL, NULL, NULL);
  int16_t p_yaw = pid_update(&pid_yaw, err_yaw, rate_yaw, NULL, NULL, NULL);
#endif

  // Mixer deltas, smoothed before the throttle is added so throttle response
  // stays immediate while the stabilisation output is clean.
  int16_t s_fr = smooth_delta(0, (int16_t)(-p_pitch - p_roll + p_yaw));
  int16_t s_fl = smooth_delta(1, (int16_t)(-p_pitch + p_roll - p_yaw));
  int16_t s_br = smooth_delta(2, (int16_t)(p_pitch - p_roll - p_yaw));
  int16_t s_bl = smooth_delta(3, (int16_t)(p_pitch + p_roll + p_yaw));
  mix_primed = 1;

  motor_fr = clamp_motor((int16_t)(throttle_base + s_fr));
  motor_fl = clamp_motor((int16_t)(throttle_base + s_fl));
  motor_br = clamp_motor((int16_t)(throttle_base + s_br));
  motor_bl = clamp_motor((int16_t)(throttle_base + s_bl));
}

void control_get_motors(int16_t *m) {
  m[0] = motor_br;
  m[1] = motor_fl;
  m[2] = motor_fr;
  m[3] = motor_bl;
}
