#ifndef TWI_H
#define TWI_H

#include <stdint.h>

void twi_init(void);
uint8_t twi_start(void);
void twi_stop(void);
uint8_t twi_write(uint8_t data);
uint8_t twi_read(uint8_t ack);

#endif
