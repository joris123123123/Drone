#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include <stdint.h>

// Motor control constants
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_LPF 64
#define MOTOR_SLEW 500

// PID constants
#define I_LIMIT   30000
#define OUT_LIMIT 800
#define D_FILTER_ALPHA 64

// Control structure
typedef struct {
    int16_t kp, ki, kd;
    int32_t integral;
    int16_t prev_der;
    int16_t prev_meas;
} pid_axis_t;

// Control functions
void control_init(void);
void control_reset(void);
void control_loop(const uint16_t *rc, int16_t gx, int16_t gy, int16_t gz);
void control_get_motors(int16_t *m);

// PID functions
void pid_reset(pid_axis_t *p);
int16_t pid_update(pid_axis_t *p, int32_t error, int16_t measurement,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out);

// Attitude functions
void attitude_init(void);
void attitude_update(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
int16_t attitude_pitch(void);
int16_t attitude_roll(void);
int16_t attitude_rate_pitch(void);
int16_t attitude_rate_roll(void);

// External control variables
extern int32_t ctrl_pitch_err;
extern int16_t ctrl_pitch_out;
extern int16_t ctrl_pitch_p;
extern int32_t ctrl_pitch_i;
extern int16_t ctrl_pitch_d;

#endif
