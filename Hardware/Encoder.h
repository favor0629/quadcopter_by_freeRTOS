#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

typedef enum
{
    ENCODER_LEFT = 1,
    ENCODER_RIGHT
}Encoder_t;

void Encoder_Init(void);
int16_t Encoder_Get(uint8_t n);

#endif /* __ENCODER_H */