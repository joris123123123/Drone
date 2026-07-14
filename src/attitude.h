#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

void attitude_init(void);
void attitude_update(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz);
int16_t attitude_pitch(void);
int16_t attitude_roll(void);

#endif
