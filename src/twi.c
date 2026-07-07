#include "twi.h"
#include <avr/io.h>

#define TWI_TIMEOUT 30000

void twi_init(void) {
    TWSR = 0;
    TWBR = 12;
}

uint8_t twi_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    uint16_t tout = 0;
    while (!(TWCR & (1 << TWINT)))
        if (++tout > TWI_TIMEOUT)
            return 1;
    return 0;
}

void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    uint16_t tout = 0;
    while (!(TWCR & (1 << TWINT)))
        if (++tout > TWI_TIMEOUT)
            return 1;
    return 0;
}

uint8_t twi_read(uint8_t ack, uint8_t *data) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    uint16_t tout = 0;
    while (!(TWCR & (1 << TWINT)))
        if (++tout > TWI_TIMEOUT)
            return 1;
    *data = TWDR;
    return 0;
}
