#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <stdint.h>

// =============================
// MOTOR CONFIGURATION
// =============================

// Motor mapping (order: back-right, front-left, front-right, back-left)
#define MOTOR_BACK_RIGHT 0
#define MOTOR_FRONT_LEFT 1
#define MOTOR_FRONT_RIGHT 2
#define MOTOR_BACK_LEFT 3

// Motor control limits
#define MOTOR_MIN_PULSE 1000
#define MOTOR_MAX_PULSE 2000
#define MOTOR_IDLE_PULSE 1000

// =============================
// RC INPUT CONFIGURATION  
// =============================

// RC channel mapping
#define RC_CHANNEL_ROLL   0
#define RC_CHANNEL_PITCH  1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW    3
#define RC_CHANNEL_ARM    4
#define RC_CHANNEL_MODE   5

// RC pulse limits
#define RC_MIN_PULSE 1000
#define RC_MAX_PULSE 2000
#define RC_CENTER_PULSE 1500
#define RC_ARM_THRESHOLD 1800
#define RC_DISARM_THRESHOLD 1500
#define RC_THROTTLE_LOW 1100

// =============================
// PID PARAMETERS
// =============================

// PID structure for control axes
typedef struct {
    int16_t kp;     // Proportional gain
    int16_t ki;     // Integral gain  
    int16_t kd;     // Derivative gain
    int32_t integral;
    int16_t prev_der;
    int16_t prev_meas;
} pid_axis_t;

// PID tuning parameters (pitch axis - can be adjusted for different hardware)
#define PID_PITCH_KP 10
#define PID_PITCH_KI 0
#define PID_PITCH_KD 0

#define PID_ROLL_KP 0
#define PID_ROLL_KI 0  
#define PID_ROLL_KD 0

#define PID_YAW_KP 0
#define PID_YAW_KI 0
#define PID_YAW_KD 0

// PID limits
#define PID_I_LIMIT   30000
#define PID_OUT_LIMIT 800
#define PID_D_FILTER_ALPHA 64

// =============================
// SENSOR CONFIGURATION
// =============================

// MPU6050 gyro scale factor (LSB per deg/s)
#define MPU_GYRO_SCALE 131

// Accelerometer calibration parameters
#define ACCEL_MIN2 (3072L * 3072L) // (0.75 g)^2  
#define ACCEL_MAX2 (5120L * 5120L) // (1.25 g)^2

// Attitude filter parameters
#define ACCEL_ALPHA 8      // Accelerometer drift correction strength
#define BIAS_KI 4          // Bias trim rate
#define BIAS_LIMIT_Q8 (2000L * 256) // +-15 deg/s limit

// =============================
// CONTROL LOOP PARAMETERS
// =============================

// Control loop frequency (Hz)
#define CTRL_RATE_HZ 100

// Motor smoothing parameters
#define MOTOR_LPF 128      // First-order coefficient (/256 per 100 Hz loop)
#define MOTOR_SLEW 100     // Max change per loop in us

// Rate command scale (50 -> ~190 deg/s max)
#define RATE_SCALE 50

// =============================
// FUNCTION PROTOTYPES
// =============================

// Motor control functions
void motor_init(void);
void motor_set_pulse(uint8_t motor_index, uint16_t pulse_width);
void motor_set_all(const int16_t *pulses);

// RC input functions  
uint16_t rc_get_channel(uint8_t channel);
uint8_t rc_valid(void);

// PID functions
void pid_reset(pid_axis_t *p);
int16_t pid_update(pid_axis_t *p, int32_t error, int16_t measurement,
                   int16_t *p_out, int32_t *i_out, int16_t *d_out);

// Control functions
void control_init(void);
void control_reset(void);
void control_update(const uint16_t *rc, int16_t gx, int16_t gy, int16_t gz);
void control_get_motor_outputs(int16_t *motors);

// Attitude estimation functions
void attitude_init(void);
void attitude_update(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
int16_t attitude_get_pitch(void);
int16_t attitude_get_roll(void);
int16_t attitude_get_rate_pitch(void);
int16_t attitude_get_rate_roll(void);

// Hardware interface functions
void hardware_init(void);
uint8_t hardware_check_sensors(void);

#endif // HARDWARE_INTERFACE_H
