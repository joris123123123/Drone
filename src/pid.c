#include "pid.h"

#define I_LIMIT   8000
#define OUT_LIMIT 400

#define D_FILTER_ALPHA 192

void pid_reset(pid_axis_t *p) {
    p->integral = 0;
    p->prev_err = 0;
    p->prev_der = 0;
}

int16_t pid_update(pid_axis_t *p, int16_t error,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out) {
    int32_t p_term = (int32_t)error * p->kp;

    int16_t raw_der = error - p->prev_err;
    p->prev_err = error;
    int16_t der = (int16_t)(((int32_t)raw_der * D_FILTER_ALPHA +
                             (int32_t)p->prev_der * (256 - D_FILTER_ALPHA)) >> 8);
    p->prev_der = der;

    int32_t d_term = (int32_t)der * p->kd;

    int32_t out_raw = (p_term + d_term) >> 8;

    if (out_raw > OUT_LIMIT || out_raw < -OUT_LIMIT) {
        if ((p->integral > 0 && error > 0) ||
            (p->integral < 0 && error < 0))
            p->integral -= error;
    } else {
        p->integral += error;
    }

    if (p->integral > I_LIMIT) p->integral = I_LIMIT;
    if (p->integral < -I_LIMIT) p->integral = -I_LIMIT;
    int32_t i_term = p->integral * p->ki;

    int32_t out = (p_term + i_term + d_term) >> 8;
    if (out > OUT_LIMIT) out = OUT_LIMIT;
    if (out < -OUT_LIMIT) out = -OUT_LIMIT;

    if (p_out) *p_out = (int16_t)(p_term >> 8);
    if (i_out) *i_out = (i_term >> 8);
    if (d_out) *d_out = (int16_t)(d_term >> 8);

    return (int16_t)out;
}
