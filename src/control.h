#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

typedef struct {
  int16_t kp, ki, kd;
  int32_t integral;
  int16_t prev_err;
} pid_axis_t;

void pid_reset(pid_axis_t *p);
int16_t pid_update(pid_axis_t *p, int16_t error,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out);

void control_init(void);
void control_reset(void);
void control_loop(const uint16_t *rc, int16_t gx, int16_t gy, int16_t gz);
void control_get_motors(int16_t *m);

extern int16_t ctrl_pitch_err;
extern int16_t ctrl_pitch_out;
extern int16_t ctrl_pitch_p;
extern int32_t ctrl_pitch_i;
extern int16_t ctrl_pitch_d;

#endif
