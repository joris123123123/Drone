#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>

#define SBUS_MIN 173
#define SBUS_MAX 1811
#define SBUS_MID 992
#define SBUS_CHANNELS 16
#define SBUS_FRAME_SIZE 25

void sbus_init(void);
uint8_t sbus_read(uint16_t *channels);
uint8_t sbus_failsafe(void);
uint16_t sbus_map(uint16_t val, uint16_t out_min, uint16_t out_max);

#endif
