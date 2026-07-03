#include "twi.h"
#include <avr/io.h>

void twi_init(void) {
  TWSR = 0;
  TWBR = 32;
}

void twi_start(void) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

void twi_stop(void) {
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

void twi_write(uint8_t data) {
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

uint8_t twi_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack ? (1<<TWEA) : 0);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}
