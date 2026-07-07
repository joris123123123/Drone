#include "control.h"
#include "gyro.h"
#include "input.h"
#include "pins.h"
#include "sbus.h"
#include "twi.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_IDLE 1000
#define PULSE_ARM 1800
#define PULSE_DISARM 1500
#define THR_LOW 1100

#define FS_TIMEOUT 50
#define CTRL_RATE_HZ 100
#define CTRL_DIV 500

#define CALIB_SAMPLES 128

#define US_PER_TICK 20
#define MOTOR_STEP US_PER_TICK

volatile uint8_t esc_fall[4] = {50, 50, 50, 50};
volatile uint8_t ctrl_flag = 0;

// serial_init() entfällt – sbus_init() konfiguriert USART0 für 100000 baud 8E2

static void esc_init(void) {
  DDRD |= (1 << ESC_BR) | (1 << ESC_FL) | (1 << ESC_FR);
  DDRB |= (1 << ESC_BL);
}

static void timer1_init(void) { TCCR1B |= (1 << CS11); }

static void timer2_init(void) {
  TCCR2A |= (1 << WGM21);
  OCR2A = 9;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21) | (1 << CS20);
}

ISR(TIMER2_COMPA_vect) {
  static uint16_t tick = 0;
  tick++;
    if (tick >= 1000) {
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
  if (div >= CTRL_DIV) {
    div = 0;
    ctrl_flag = 1;
  }
}

static void motors_idle(void) {
  cli();
  esc_fall[0] = PULSE_IDLE / MOTOR_STEP;
  esc_fall[1] = PULSE_IDLE / MOTOR_STEP;
  esc_fall[2] = PULSE_IDLE / MOTOR_STEP;
  esc_fall[3] = PULSE_IDLE / MOTOR_STEP;
  sei();
}

static void motors_set(const int16_t *m) {
  uint8_t v;
  cli();
  for (uint8_t i = 0; i < 4; i++) {
    v = (uint8_t)(m[i] / MOTOR_STEP);
    if (v < 50)
      v = 50;
    if (v > 100)
      v = 100;
    esc_fall[i] = v;
  }
  sei();
}

static uint8_t signal_valid(const uint16_t *rc) {
  for (uint8_t i = 0; i < 4; i++) {
    if (rc[i] < 800 || rc[i] > 2200)
      return 0;
  }
  return 1;
}

static void delay_ms(uint16_t ms) {
  while (ms--) {
    for (volatile uint16_t i = 0; i < 4000; i++)
      ;
  }
}

int main(void) {
  sbus_init();
  DDRB |= (1 << PB5);
  PORTB |= (1 << PB5);

  printf("Drone v2 init...\r\n");
  twi_init();

  if (!gyro_init()) {
    printf("GYRO FAIL\r\n");
  } else {
    printf("GYRO OK\r\n");
  }
  printf("Calibrating...\r\n");

  control_init();
  esc_init();
  timer1_init();

  mpu_data_t gyro;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    gyro_read(&gyro);
    sum_gx += gyro.gx;
    sum_gy += gyro.gy;
    sum_gz += gyro.gz;
  }

  int16_t gx_off = (int16_t)(sum_gx / CALIB_SAMPLES);
  int16_t gy_off = (int16_t)(sum_gy / CALIB_SAMPLES);
  int16_t gz_off = (int16_t)(sum_gz / CALIB_SAMPLES);
  printf("Calib done. off=%d %d %d\r\n", gx_off, gy_off, gz_off);

  uint16_t rc[6] = {1500, 1500, 1500, 1500, 1500, 1500};
  int16_t motors[4] = {1000, 1000, 1000, 1000};
  uint8_t armed = 0;
  uint16_t fs_count = 0;
  uint16_t prev_tcnt = 0;

  sei();
  timer2_init();

  // test ESCs independently to check position
  // test BR ESC
  /*delay_ms(1000);
  printf("BR ESC test 1100us...\r\n");
  esc_fall[0] = 110;
  delay_ms(500);
  printf("BR ESC tested");
  esc_fall[0] = 100;
  // test BL ESC
  printf("BL ESC test 1100us...\r\n");
  esc_fall[1] = 110;
  delay_ms(500);
  printf("BL ESC tested");
  esc_fall[1] = 100;
  // test FR ESC
  printf("FR ESC test 1100us...\r\n");
  esc_fall[2] = 110;
  delay_ms(500);
  printf("FR ESC tested");
  esc_fall[2] = 100;
  // test FL ESC
  printf("FL ESC test 1200us...\r\n");
  esc_fall[3] = 120;
  delay_ms(500);
  printf("FL ESC tested");
  esc_fall[3] = 100;
  // test all ESCs
  printf("ESC test all 1100us...\r\n");
  esc_fall[0] = 110;
  esc_fall[1] = 110;
  esc_fall[2] = 110;
  esc_fall[3] = 110;
  delay_ms(1000);
  esc_fall[0] = 100;
  esc_fall[1] = 100;
  esc_fall[2] = 100;
  esc_fall[3] = 100;
  printf("ESC test done\r\n");*/

  delay_ms(100);
  printf("init done \n");

  uint32_t acc_ticks = 0;

  while (1) {
    if (!ctrl_flag)
      continue;
    ctrl_flag = 0;

    uint16_t t = TCNT1;
    uint16_t loop_ticks = t - prev_tcnt;
    prev_tcnt = t;
    acc_ticks += loop_ticks;

    uint16_t sbus_ch[16];
    if (sbus_read(sbus_ch)) {
      rc[CH_ROLL] = sbus_map(sbus_ch[0], 1000, 2000);
      rc[CH_PITCH] = sbus_map(sbus_ch[1], 1000, 2000);
      rc[CH_THR] = sbus_map(sbus_ch[2], 1000, 2000);
      rc[CH_YAW] = sbus_map(sbus_ch[3], 1000, 2000);
      rc[CH_MODE] = sbus_map(sbus_ch[4], 1000, 2000);
      rc[CH_ARM] = sbus_map(sbus_ch[5], 1000, 2000);
    }

    int16_t gx = 0, gy = 0, gz = 0;
    if (!signal_valid(rc) || sbus_failsafe()) {
      fs_count++;
      if (fs_count > FS_TIMEOUT) {
        armed = 0;
        motors_idle();
      }
      goto alldone;
    }
    fs_count = 0;

    gyro_read_gyro(&gyro);
    gx = gyro.gx - gx_off;
    gy = gyro.gy - gy_off;
    gz = gyro.gz - gz_off;

    if (rc[CH_ARM] < PULSE_DISARM) {
      if (armed) {
        armed = 0;
        motors_idle();
      }
    }

    if (!armed) {
      if (rc[CH_ARM] > PULSE_ARM && rc[CH_THR] < THR_LOW) {
        armed = 1;
        control_reset();
      }
      motors_idle();
    } else {
      control_loop(rc, gx, gy, gz);
      control_get_motors(motors);
      motors_set(motors);
    }

  alldone:
    (void)gy;
    static uint8_t blink = 0;
    if (++blink >= 125) {
      blink = 0;
      PORTB ^= (1 << PB5);
    }

    static uint8_t dbg = 0;
    if (++dbg >= 5) {
      dbg = 0;
      gyro_read_accel(&gyro);
      uint16_t avg = (uint16_t)(acc_ticks / 5);
      acc_ticks = 0;
      printf("ax=%+5d ay=%+5d az=%+5d "
             "gx=%+5d gy=%+5d gz=%+5d %s %u\r\n",
             gyro.ax, gyro.ay, gyro.az, gx, gy, gz, armed ? "ARM" : "DIS",
             avg);
    }
  }
}
