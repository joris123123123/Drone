#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>

#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THR 2
#define CH_YAW 3
#define CH_MODE 4
#define CH_ARM 5

void input_init(void);
uint16_t input_read(uint8_t ch);

#endif
