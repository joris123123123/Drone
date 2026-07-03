#include "input.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>

static volatile uint16_t start_tick[6];
static volatile uint16_t pulse_ticks[6];

void input_init(void) {
  DDRC &= ~0x0F;
  DDRB &= ~((1 << PB1) | (1 << PB2));
  PORTC |= 0x0F;
  PORTB |= (1 << PB1) | (1 << PB2);

  PCICR |= (1 << PCIE1) | (1 << PCIE0);
  PCMSK1 |= 0x0F;
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2);
}

uint16_t input_read(uint8_t ch) {
  uint8_t sreg = SREG;
  cli();
  uint16_t v = pulse_ticks[ch];
  SREG = sreg;
  return v;
}

ISR(PCINT1_vect) {
  uint8_t p = PINC;
  uint16_t now = TCNT1;
  if (p & (1 << PC0))
    start_tick[0] = now;
  else
    pulse_ticks[0] = now - start_tick[0];
  if (p & (1 << PC1))
    start_tick[1] = now;
  else
    pulse_ticks[1] = now - start_tick[1];
  if (p & (1 << PC2))
    start_tick[2] = now;
  else
    pulse_ticks[2] = now - start_tick[2];
  if (p & (1 << PC3))
    start_tick[3] = now;
  else
    pulse_ticks[3] = now - start_tick[3];
}

ISR(PCINT0_vect) {
  uint8_t p = PINB;
  uint16_t now = TCNT1;
  if (p & (1 << PB1))
    start_tick[4] = now;
  else
    pulse_ticks[4] = now - start_tick[4];
  if (p & (1 << PB2))
    start_tick[5] = now;
  else
    pulse_ticks[5] = now - start_tick[5];
}
