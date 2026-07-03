#include "control.h"
#include "input.h"
#include <avr/io.h>
#include <stddef.h>

#include "pid.h"

#define MOTOR_MIN 1000
#define MOTOR_MAX 2000

static pid_axis_t pid_roll, pid_pitch, pid_yaw;

static int16_t motor_br, motor_fl, motor_fr, motor_bl;
static int16_t throttle_base;

int16_t ctrl_pitch_err = 0;
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
  pid_roll.kp = 0;
  pid_roll.ki = 0;
  pid_roll.kd = 0;
  pid_reset(&pid_roll);

  pid_pitch.kp = 768;
  pid_pitch.ki = 16;
  pid_pitch.kd = 256;
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
}

void control_loop(const uint16_t *rc, int16_t gx, int16_t gy, int16_t gz) {
  throttle_base = rc[CH_THR];

  int16_t rc_roll = (int16_t)(rc[CH_ROLL] - 1500);
  int16_t rc_pitch = (int16_t)(rc[CH_PITCH] - 1500);
  int16_t rc_yaw = (int16_t)(rc[CH_YAW] - 1500);

  int16_t err_roll = rc_roll * 2 + gx;
  int16_t err_pitch = rc_pitch * 2 + gy;
  int16_t err_yaw = rc_yaw * 2 + gz;

  int16_t p_roll = pid_update(&pid_roll, err_roll, NULL, NULL, NULL);
  int16_t p_pitch = pid_update(&pid_pitch, err_pitch, &ctrl_pitch_p,
                               &ctrl_pitch_i, &ctrl_pitch_d);
  int16_t p_yaw = pid_update(&pid_yaw, err_yaw, NULL, NULL, NULL);

  ctrl_pitch_err = err_pitch;
  ctrl_pitch_out = p_pitch;

  motor_fr = throttle_base - p_pitch - p_roll + p_yaw;
  motor_fl = throttle_base - p_pitch + p_roll - p_yaw;
  motor_br = throttle_base + p_pitch - p_roll - p_yaw;
  motor_bl = throttle_base + p_pitch + p_roll + p_yaw;

  motor_fr = clamp_motor(motor_fr);
  motor_fl = clamp_motor(motor_fl);
  motor_br = clamp_motor(motor_br);
  motor_bl = clamp_motor(motor_bl);
}

void control_get_motors(int16_t *m) {
  m[0] = motor_br;
  m[1] = motor_fl;
  m[2] = motor_fr;
  m[3] = motor_bl;
}
