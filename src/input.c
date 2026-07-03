#include "input.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>

static volatile uint16_t start_tick[6];
static volatile uint16_t pulse_ticks[6];

static uint8_t last_pinc;
static uint8_t last_pinb;

void input_init(void) {
    DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3));
    DDRB &= ~((1 << PB1) | (1 << PB2));
    PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
    PORTB |= (1 << PB1) | (1 << PB2);

    PCICR |= (1 << PCIE1) | (1 << PCIE0);
    PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
    PCMSK0 |= (1 << PCINT1) | (1 << PCINT2);

    last_pinc = PINC;
    last_pinb = PINB;
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
    uint8_t changed = p ^ last_pinc;
    uint16_t now = TCNT1;

    if (changed & (1 << PC0)) {
        if (p & (1 << PC0))
            start_tick[0] = now;
        else
            pulse_ticks[0] = now - start_tick[0];
    }
    if (changed & (1 << PC1)) {
        if (p & (1 << PC1))
            start_tick[1] = now;
        else
            pulse_ticks[1] = now - start_tick[1];
    }
    if (changed & (1 << PC2)) {
        if (p & (1 << PC2))
            start_tick[2] = now;
        else
            pulse_ticks[2] = now - start_tick[2];
    }
    if (changed & (1 << PC3)) {
        if (p & (1 << PC3))
            start_tick[3] = now;
        else
            pulse_ticks[3] = now - start_tick[3];
    }

    last_pinc = p;
}

ISR(PCINT0_vect) {
    uint8_t p = PINB;
    uint8_t changed = p ^ last_pinb;
    uint16_t now = TCNT1;

    if (changed & (1 << PB1)) {
        if (p & (1 << PB1))
            start_tick[4] = now;
        else
            pulse_ticks[4] = now - start_tick[4];
    }
    if (changed & (1 << PB2)) {
        if (p & (1 << PB2))
            start_tick[5] = now;
        else
            pulse_ticks[5] = now - start_tick[5];
    }

    last_pinb = p;
}
