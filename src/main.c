#include "control.h"
#include "gyro.h"
#include "input.h"
#include "pins.h"
#include "twi.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_IDLE 1000
#define PULSE_ARM 1800
#define PULSE_DISARM 1500
#define THR_LOW 1100

volatile uint8_t esc_fall[4] = {100, 100, 100, 100};
volatile uint8_t ctrl_flag = 0;

static void esc_init(void) {
  DDRD |= (1 << ESC_BR) | (1 << ESC_FL) | (1 << ESC_FR);
  DDRB |= (1 << ESC_BL);
}

static void timer1_init(void) { TCCR1B |= (1 << CS11); }

static void timer2_init(void) {
  TCCR2A |= (1 << WGM21);
  OCR2A = 4;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21) | (1 << CS20);
}

ISR(TIMER2_COMPA_vect) {
  static uint16_t tick = 0;
  tick++;
  if (tick >= 2000) {
    tick = 0;
    PORTD |= (1 << ESC_BR) | (1 << ESC_FL) | (1 << ESC_FR);
    PORTB |= (1 << ESC_BL);
  }
  if (tick == esc_fall[0])
    PORTD &= ~(1 << ESC_BR);
  if (tick == esc_fall[1])
    PORTD &= ~(1 << ESC_FL);
  if (tick == esc_fall[2])
    PORTD &= ~(1 << ESC_FR);
  if (tick == esc_fall[3])
    PORTB &= ~(1 << ESC_BL);
  static uint16_t div = 0;
  div++;
  if (div >= 400) {
    div = 0;
    ctrl_flag = 1;
  }
}

static void motors_idle(void) {
  cli();
  esc_fall[0] = PULSE_IDLE / 10;
  esc_fall[1] = PULSE_IDLE / 10;
  esc_fall[2] = PULSE_IDLE / 10;
  esc_fall[3] = PULSE_IDLE / 10;
  sei();
}

static void motors_set(const int16_t *m) {
  uint8_t v;
  cli();
  for (uint8_t i = 0; i < 4; i++) {
    v = (uint8_t)(m[i] / 10);
    if (v < 100)
      v = 100;
    if (v > 200)
      v = 200;
    esc_fall[i] = v;
  }
  sei();
}

static uint8_t signal_valid(uint16_t *rc) {
  for (uint8_t i = 0; i < 4; i++) {
    if (rc[i] < 800 || rc[i] > 2200)
      return 0;
  }
  return 1;
}

int main(void) {
  twi_init();
  gyro_init();
  input_init();
  control_init();
  esc_init();
  timer1_init();

  mpu_data_t gyro;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  for (uint16_t i = 0; i < 256; i++) {
    gyro_read(&gyro);
    sum_gx += gyro.gx;
    sum_gy += gyro.gy;
    sum_gz += gyro.gz;
  }

  int16_t gx_off = (int16_t)(sum_gx / 256);
  int16_t gy_off = (int16_t)(sum_gy / 256);
  int16_t gz_off = (int16_t)(sum_gz / 256);

  uint16_t rc[6];
  int16_t motors[4];
  uint8_t armed = 0;
  uint16_t fs_count = 0;

  sei();
  timer2_init();

  while (1) {
    if (!ctrl_flag)
      continue;
    ctrl_flag = 0;

    rc[CH_ROLL] = input_read(CH_ROLL) / 2;
    rc[CH_PITCH] = input_read(CH_PITCH) / 2;
    rc[CH_THR] = input_read(CH_THR) / 2;
    rc[CH_YAW] = input_read(CH_YAW) / 2;
    rc[CH_MODE] = input_read(CH_MODE) / 2;
    rc[CH_ARM] = input_read(CH_ARM) / 2;

    if (signal_valid(rc)) {
      fs_count = 0;
    } else {
      fs_count++;
      if (fs_count > 50) {
        armed = 0;
        motors_idle();
        continue;
      }
      continue;
    }

    gyro_read(&gyro);
    int16_t gx = gyro.gx - gx_off;
    int16_t gy = gyro.gy - gy_off;
    int16_t gz = gyro.gz - gz_off;

    if (!armed) {
      if (rc[CH_ARM] > PULSE_ARM && rc[CH_THR] < THR_LOW) {
        armed = 1;
        control_reset();
      }
      if (!armed) {
        motors_idle();
        continue;
      }
    }

    if (rc[CH_ARM] < PULSE_DISARM) {
      armed = 0;
      motors_idle();
      continue;
    }

    control_loop(rc, gx, gy, gz);
    control_get_motors(motors);
    motors_set(motors);
  }
}
