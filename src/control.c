#include "control.h"
#include "input.h"
#include <avr/io.h>

#define I_LIMIT 8000
#define OUT_LIMIT 400

static pid_axis_t pid_roll, pid_pitch, pid_yaw;

static int16_t motor_br, motor_fl, motor_fr, motor_bl;
static int16_t throttle_base;

void pid_reset(pid_axis_t *p) {
  p->integral = 0;
  p->prev_err = 0;
}

int16_t pid_update(pid_axis_t *p, int16_t error) {
  int32_t p_term = (int32_t)error * p->kp;
  p->integral += error;
  if (p->integral > I_LIMIT)
    p->integral = I_LIMIT;
  if (p->integral < -I_LIMIT)
    p->integral = -I_LIMIT;
  int32_t i_term = p->integral * p->ki;
  int16_t d = error - p->prev_err;
  p->prev_err = error;
  int32_t d_term = (int32_t)d * p->kd;
  int32_t out = (p_term + i_term + d_term) >> 8;
  if (out > OUT_LIMIT)
    out = OUT_LIMIT;
  if (out < -OUT_LIMIT)
    out = -OUT_LIMIT;
  return (int16_t)out;
}

void control_init(void) {
  pid_roll.kp = 768;
  pid_roll.ki = 16;
  pid_roll.kd = 256;
  pid_reset(&pid_roll);

  pid_pitch.kp = 768;
  pid_pitch.ki = 16;
  pid_pitch.kd = 256;
  pid_reset(&pid_pitch);

  pid_yaw.kp = 512;
  pid_yaw.ki = 8;
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

  int16_t p_roll = pid_update(&pid_roll, err_roll);
  int16_t p_pitch = pid_update(&pid_pitch, err_pitch);
  int16_t p_yaw = pid_update(&pid_yaw, err_yaw);

  motor_fr = throttle_base - p_pitch - p_roll + p_yaw;
  motor_fl = throttle_base - p_pitch + p_roll - p_yaw;
  motor_br = throttle_base + p_pitch - p_roll - p_yaw;
  motor_bl = throttle_base + p_pitch + p_roll + p_yaw;

  if (motor_fr < 1000) {
    motor_fr = 1000;
  }
  if (motor_fr > 2000) {
    motor_fr = 2000;
  }
  if (motor_fl < 1000) {
    motor_fl = 1000;
  }
  if (motor_fl > 2000) {
    motor_fl = 2000;
  }
  if (motor_br < 1000) {
    motor_br = 1000;
  }
  if (motor_br > 2000) {
    motor_br = 2000;
  }
  if (motor_bl < 1000) {
    motor_bl = 1000;
  }
  if (motor_bl > 2000) {
    motor_bl = 2000;
  }
}

void control_get_motors(int16_t *m) {
  m[0] = motor_br;
  m[1] = motor_fl;
  m[2] = motor_fr;
  m[3] = motor_bl;
}
