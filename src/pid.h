#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int16_t kp, ki, kd;
    int32_t integral;
    int16_t prev_der;
    int16_t prev_meas;
} pid_axis_t;

void pid_reset(pid_axis_t *p);
int16_t pid_update(pid_axis_t *p, int16_t error, int16_t measurement,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out);

#endif
