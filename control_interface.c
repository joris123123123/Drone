#include "control_interface.h"
#include <stdint.h>

// PID structure for pitch control (as defined in original code)
static pid_axis_t pid_roll, pid_pitch, pid_yaw;

// Motor outputs
static int16_t motor_br, motor_fl, motor_fr, motor_bl;
static int16_t throttle_base;

// Smoothed mixer deltas
static int16_t mix_filt[4];
static uint8_t mix_primed;

// Control variables
int32_t ctrl_pitch_err = 0;
int16_t ctrl_pitch_out = 0;
int16_t ctrl_pitch_p = 0;
int32_t ctrl_pitch_i = 0;
int16_t ctrl_pitch_d = 0;

// Helper function to clamp motor values
static int16_t clamp_motor(int16_t v) {
    if (v < MOTOR_MIN)
        return MOTOR_MIN;
    if (v > MOTOR_MAX)
        return MOTOR_MAX;
    return v;
}

void control_init(void) {
    // Initialize PID controllers with default values from original code
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
    mix_primed = 0;
}

void pid_reset(pid_axis_t *p) {
    p->integral = 0;
    p->prev_der = 0;
    p->prev_meas = 0;
}

int16_t pid_update(pid_axis_t *p, int32_t error, int16_t measurement,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out) {
    // Integral accumulates before output
    p->integral += error;
    if (p->integral > I_LIMIT) p->integral = I_LIMIT;
    if (p->integral < -I_LIMIT) p->integral = -I_LIMIT;

    int32_t p_term = error * p->kp;
    int32_t i_term = p->integral * p->ki;

    // Derivative on measurement (not error) to avoid derivative kick
    int16_t raw_der = measurement - p->prev_meas;
    p->prev_meas = measurement;
    int16_t der = (int16_t)(((int32_t)raw_der * D_FILTER_ALPHA +
                             (int32_t)p->prev_der * (256 - D_FILTER_ALPHA)) >> 8);
    p->prev_der = der;
    int32_t d_term = (int32_t)der * p->kd;

    int32_t out = (p_term + i_term + d_term) >> 8;

    // Anti-windup: undo integral if saturated
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

void control_loop(const uint16_t *rc, int16_t gx, int16_t gy, int16_t gz) {
    throttle_base = rc[2]; // Assuming channel 2 is throttle

    // Measured body rates
    int16_t rate_pitch = -gy;
    int16_t rate_roll = -gx;
    int16_t rate_yaw = -gz;

    // Pitch control
    int32_t err_pitch = (int32_t)(rc[1] - 1500) * 50 + rate_pitch; // Assuming channel 1 is pitch
    int16_t p_pitch = pid_update(&pid_pitch, err_pitch, rate_pitch, &ctrl_pitch_p,
                                  &ctrl_pitch_i, &ctrl_pitch_d);

    ctrl_pitch_err = err_pitch;
    ctrl_pitch_out = p_pitch;

    // Roll and yaw are not implemented in this simplified version
    int16_t p_roll = 0;
    int16_t p_yaw = 0;

    // Mixer deltas (simplified for pitch-only control)
    int16_t s_fr = (int16_t)(-p_pitch - p_roll + p_yaw);
    int16_t s_fl = (int16_t)(-p_pitch + p_roll - p_yaw);
    int16_t s_br = (int16_t)(p_pitch - p_roll - p_yaw);
    int16_t s_bl = (int16_t)(p_pitch + p_roll + p_yaw);

    // Apply motor smoothing
    if (!mix_primed) {
        mix_filt[0] = s_fr;
        mix_filt[1] = s_fl;
        mix_filt[2] = s_br;
        mix_filt[3] = s_bl;
        mix_primed = 1;
    } else {
        // Apply first-order low-pass filter and slew rate limiting
        for (int i = 0; i < 4; i++) {
            int32_t step = ((int32_t)(mix_filt[i] - mix_filt[i]) * MOTOR_LPF) >> 8;
            if (step > MOTOR_SLEW)
                step = MOTOR_SLEW;
            else if (step < -MOTOR_SLEW)
                step = -MOTOR_SLEW;
            mix_filt[i] = (int16_t)(mix_filt[i] + step);
        }
    }

    // Calculate final motor outputs
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

// Attitude functions (simplified version)
void attitude_init(void) {
    // Initialize attitude tracking
}

void attitude_update(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    // Update attitude estimation
}

int16_t attitude_pitch(void) {
    return 0;
}

int16_t attitude_roll(void) {
    return 0;
}

int16_t attitude_rate_pitch(void) {
    return 0;
}

int16_t attitude_rate_roll(void) {
    return 0;
}
