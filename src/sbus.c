#include "sbus.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

#define SBUS_UBRR (F_CPU / 8 / 100000 - 1)

static volatile uint8_t buf[25];
static volatile uint8_t idx = 0;
static volatile uint8_t frame_ready = 0;
static volatile uint8_t failsafe_flag = 0;

static int uart_putchar(char c, FILE *stream) {
  if (c == '\n')
    uart_putchar('\r', stream);
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = c;
  return 0;
}

static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void sbus_init(void) {
  UCSR0A = (1 << U2X0);
  UBRR0H = 0;
  UBRR0L = SBUS_UBRR;
  UCSR0C = (1 << UPM01) | (1 << UPM00) | (1 << USBS0) | (1 << UCSZ01) |
           (1 << UCSZ00);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  stdout = &uart_out;
}

ISR(USART_RX_vect) {
  uint8_t b = UDR0;
  if (frame_ready)
    return;
  if (idx == 0 && b != 0x0F)
    return;
  buf[idx++] = b;
  if (idx == 25) {
    idx = 0;
    if (b == 0x00) {
      frame_ready = 1;
      failsafe_flag = (buf[23] >> 3) & 1;
    }
  }
}

uint8_t sbus_read(uint16_t *channels) {
  if (!frame_ready)
    return 0;

  uint8_t tmp[25];
  cli();
  frame_ready = 0;
  for (uint8_t i = 0; i < 25; i++)
    tmp[i] = buf[i];
  sei();

  channels[0] = (((uint16_t)tmp[1] | ((uint16_t)tmp[2] << 8)) & 0x07FF);
  channels[1] =
      ((((uint16_t)tmp[2] >> 3) | ((uint16_t)tmp[3] << 5)) & 0x07FF);
  channels[2] = ((((uint16_t)tmp[3] >> 6) | ((uint16_t)tmp[4] << 2) |
                  ((uint16_t)tmp[5] << 10)) &
                 0x07FF);
  channels[3] =
      ((((uint16_t)tmp[5] >> 1) | ((uint16_t)tmp[6] << 7)) & 0x07FF);
  channels[4] =
      ((((uint16_t)tmp[6] >> 4) | ((uint16_t)tmp[7] << 4)) & 0x07FF);
  channels[5] = ((((uint16_t)tmp[7] >> 7) | ((uint16_t)tmp[8] << 1) |
                  ((uint16_t)tmp[9] << 9)) &
                 0x07FF);
  channels[6] =
      ((((uint16_t)tmp[9] >> 2) | ((uint16_t)tmp[10] << 6)) & 0x07FF);
  channels[7] =
      ((((uint16_t)tmp[10] >> 5) | ((uint16_t)tmp[11] << 3)) & 0x07FF);
  channels[8] = (((uint16_t)tmp[12] | ((uint16_t)tmp[13] << 8)) & 0x07FF);
  channels[9] =
      ((((uint16_t)tmp[13] >> 3) | ((uint16_t)tmp[14] << 5)) & 0x07FF);
  channels[10] = ((((uint16_t)tmp[14] >> 6) | ((uint16_t)tmp[15] << 2) |
                   ((uint16_t)tmp[16] << 10)) &
                  0x07FF);
  channels[11] =
      ((((uint16_t)tmp[16] >> 1) | ((uint16_t)tmp[17] << 7)) & 0x07FF);
  channels[12] =
      ((((uint16_t)tmp[17] >> 4) | ((uint16_t)tmp[18] << 4)) & 0x07FF);
  channels[13] = ((((uint16_t)tmp[18] >> 7) | ((uint16_t)tmp[19] << 1) |
                   ((uint16_t)tmp[20] << 9)) &
                  0x07FF);
  channels[14] =
      ((((uint16_t)tmp[20] >> 2) | ((uint16_t)tmp[21] << 6)) & 0x07FF);
  channels[15] =
      ((((uint16_t)tmp[21] >> 5) | ((uint16_t)tmp[22] << 3)) & 0x07FF);
  return 1;
}

uint8_t sbus_failsafe(void) { return failsafe_flag; }

uint16_t sbus_map(uint16_t val, uint16_t out_min, uint16_t out_max) {
  if (val < SBUS_MIN)
    val = SBUS_MIN;
  if (val > SBUS_MAX)
    val = SBUS_MAX;
  return (uint16_t)((uint32_t)(val - SBUS_MIN) * (out_max - out_min) /
                        (SBUS_MAX - SBUS_MIN) +
                    out_min);
}
