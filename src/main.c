#include "attitude.h"
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
#define PULSE_IDLE 0
#define PULSE_ARM 1800
#define PULSE_DISARM 1500
#define THR_LOW 1100

#define CTRL_RATE_HZ 100
#define CTRL_DIV 500

#define CALIB_SAMPLES 128
#define ESC_CALIB 0 // Auf 1 setzen für ESC-Kalibrierung, danach wieder auf 0

#define US_PER_TICK 20

volatile uint16_t esc_fall[4] = {1000, 1000, 1000, 1000};
volatile uint16_t esc_next[4] = {1000, 1000, 1000, 1000};
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
    esc_fall[0] = esc_next[0];
    esc_fall[1] = esc_next[1];
    esc_fall[2] = esc_next[2];
    esc_fall[3] = esc_next[3];
    PORTD |= (1 << ESC_BR) | (1 << ESC_FL) | (1 << ESC_FR);
    PORTB |= (1 << ESC_BL);
  }
  if ((uint16_t)(tick * US_PER_TICK) >= esc_fall[0])
    PORTD &= ~(1 << ESC_BR);
  if ((uint16_t)(tick * US_PER_TICK) >= esc_fall[1])
    PORTD &= ~(1 << ESC_FL);
  if ((uint16_t)(tick * US_PER_TICK) >= esc_fall[2])
    PORTD &= ~(1 << ESC_FR);
  if ((uint16_t)(tick * US_PER_TICK) >= esc_fall[3])
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
  esc_next[0] = PULSE_IDLE;
  esc_next[1] = PULSE_IDLE;
  esc_next[2] = PULSE_IDLE;
  esc_next[3] = PULSE_IDLE;
  sei();
}

static void motors_set(const int16_t *m) {
  uint16_t v;
  cli();
  for (uint8_t i = 0; i < 4; i++) {
    v = (uint16_t)m[i];
    if (v < PULSE_MIN)
      v = PULSE_MIN;
    if (v > PULSE_MAX)
      v = PULSE_MAX;
    esc_next[i] = v;
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
    uint16_t start = TCNT1;
    while ((uint16_t)(TCNT1 - start) < 2000)
      ;
  }
}

int main(void) {
  sbus_init();
  DDRB |= (1 << PB5);
  PORTB |= (1 << PB5);

  printf("DRONE v2\r\n");
  twi_init();

  if (!gyro_init()) {
    printf("GYRO: FAIL\r\n");
  } else {
    printf("GYRO: OK\r\n");
  }
  printf("CALIB...\r\n");

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
  printf("CALIB: off=%d %d %d\r\n", gx_off, gy_off, gz_off);

  attitude_init();

  uint16_t rc[6] = {1500, 1500, 1500, 1500, 1500, 1500};
  int16_t motors[4] = {1000, 1000, 1000, 1000};
  uint8_t armed = 0;
  uint16_t prev_tcnt = 0;

  sei();
  timer2_init();

  // test ESCs independently to check position
  // test BR ESC
  /*delay_ms(1000);
  printf("BR ESC test 1100us...\r\n");
  esc_next[0] = 1100;
  delay_ms(500);
  printf("BR ESC tested\n");
  esc_next[0] = PULSE_IDLE;
  // test BL ESC
  printf("FL ESC test 1100us...\r\n");
  esc_next[1] = 1100;
  delay_ms(500);
  printf("FL ESC tested \r\n");
  esc_next[1] = PULSE_IDLE;
  // test FR ESC
  printf("FR ESC test 1100us...\r\n");
  esc_next[2] = 1100;
  delay_ms(500);
  printf("FR ESC tested 1100us \r\n");
  esc_next[2] = PULSE_IDLE;
  // test FL ESC
  printf("BL ESC test 1100us...\r\n");
  esc_next[3] = 1100;
  delay_ms(500);
  printf("BL ESC tested \r\n");
  esc_next[3] = PULSE_IDLE;
  // test all ESCs
  printf("ESC test all 1100us...\r\n");
  esc_fall[0] = 1100;
  esc_fall[1] = 1100;
  esc_fall[2] = 1100;
  esc_fall[3] = 1100;
  delay_ms(1000);
  esc_fall[0] = PULSE_IDLE;
  esc_fall[1] = PULSE_IDLE;
  esc_fall[2] = PULSE_IDLE;
  esc_fall[3] = PULSE_IDLE;
  printf("ESC test done \r\n");*/

  delay_ms(100);
  printf("READY\r\n");

  // ESC Calibration: Motor-SIGNALE ZUERST AUSSCHALTEN, dann Akku ANSTECKEN.
  // ESC piepst → MAX senden → Piep → MIN senden → Piep-Piep → fertig.
  // Danach #define ESC_CALIB wieder auf 0 setzen und neu flashen.
#if ESC_CALIB
  printf("ESC CALIB: Kein Signal...\r\n");
  esc_next[0] = 0;
  esc_next[1] = 0;
  esc_next[2] = 0;
  esc_next[3] = 0;
  delay_ms(3000);
  printf("MAX...\r\n");
  esc_next[0] = PULSE_MAX;
  esc_next[1] = PULSE_MAX;
  esc_next[2] = PULSE_MAX;
  esc_next[3] = PULSE_MAX;
  delay_ms(4000);
  printf("MIN...\r\n");
  esc_next[0] = PULSE_MIN;
  esc_next[1] = PULSE_MIN;
  esc_next[2] = PULSE_MIN;
  esc_next[3] = PULSE_MIN;
  delay_ms(4000);
  esc_next[0] = 0;
  esc_next[1] = 0;
  esc_next[2] = 0;
  esc_next[3] = 0;
  printf("CALIB done. ESC_CALIB auf 0 setzen und neu flashen.\r\n");
  while (1)
    ;
#endif

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
      rc[CH_ARM] = sbus_map(sbus_ch[4], 1000, 2000);
      rc[CH_MODE] = sbus_map(sbus_ch[5], 1000, 2000);
    }

    int16_t gx = 0, gy = 0, gz = 0;
    if (!signal_valid(rc) || sbus_failsafe()) {
      armed = 0;
      motors_idle();
      goto alldone;
    }

    gyro_read(&gyro);
    gx = gyro.gx - gx_off;
    gy = gyro.gy - gy_off;
    gz = gyro.gz - gz_off;
    attitude_update(gyro.ax, gyro.ay, gyro.az, gx, gy, gz);

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
      uint16_t avg = (uint16_t)(acc_ticks / 5);
      acc_ticks = 0;
      printf("p=%+4d r=%+4d gx=%+5d gy=%+5d gz=%+5d %s %u\r\n",
             attitude_pitch(), attitude_roll(), gx, gy, gz,
             armed ? "ARM" : "DIS", avg);
    }
  }
}
