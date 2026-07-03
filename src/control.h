#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "pid.h"

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
