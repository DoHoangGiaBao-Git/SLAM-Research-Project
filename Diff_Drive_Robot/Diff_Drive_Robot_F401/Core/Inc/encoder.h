#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/**
 * Motor Encoder Pins (STM32F401RE):
 *   Left Motor  (TIM2): PA0 (CH1), PA1 (CH2) -> Nucleo A0, A1
 *   Right Motor (TIM3): PA6 (CH1), PA7 (CH2) -> Nucleo D12, D11
 */

void Encoder_Init(void);
int32_t Encoder_GetLeft(void);
int32_t Encoder_GetRight(void);
void Encoder_Read(int32_t *left, int32_t *right);
void Encoder_Reset(void);

#endif /* ENCODER_H */
