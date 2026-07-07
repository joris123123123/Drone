#include "pid.h"

#define I_LIMIT   30000
#define OUT_LIMIT 800

#define D_FILTER_ALPHA 64

void pid_reset(pid_axis_t *p) {
    p->integral = 0;
    p->prev_der = 0;
    p->prev_meas = 0;
}

int16_t pid_update(pid_axis_t *p, int16_t error, int16_t measurement,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out) {
    // Integral accumulates before output
    p->integral += error;
    if (p->integral > I_LIMIT) p->integral = I_LIMIT;
    if (p->integral < -I_LIMIT) p->integral = -I_LIMIT;

    int32_t p_term = (int32_t)error * p->kp;
    int32_t i_term = p->integral * p->ki;

    // Derivative on measurement (not error) to avoid derivative kick
    int16_t raw_der = measurement - p->prev_meas;
    p->prev_meas = measurement;
    int16_t der = (int16_t)(((int32_t)raw_der * D_FILTER_ALPHA +
                             (int32_t)p->prev_der * (256 - D_FILTER_ALPHA)) >> 8);
    p->prev_der = der;
    int32_t d_term = (int32_t)der * p->kd;

    int32_t out = (p_term + i_term + d_term) >> 8;

    // Anti-windup: undo integral if saturated (check pre-clamp value)
    if (out > OUT_LIMIT) {
        p->integral -= error;
        out = OUT_LIMIT;
    } else if (out < -OUT_LIMIT) {
        p->integral -= error;
        out = -OUT_LIMIT;
    }

    if (p_out) *p_out = (int16_t)(p_term >> 8);
    if (i_out) *i_out = (i_term >> 8);
    if (d_out) *d_out = (int16_t)(d_term >> 8);

    return (int16_t)out;
}
