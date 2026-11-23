#include "encoder.h"
#include "stm32f4xx.h"

static int32_t right_total = 0;
static int16_t last_tim3   = 0;

void Encoder_Init(void) {
    // 1. Enable peripheral clocks for GPIOA, TIM2, and TIM3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    // 2. Configure GPIOA: PA0, PA1 (AF1 for TIM2) and PA6, PA7 (AF2 for TIM3)
    // Clear MODER, PUPDR, and AFR fields first to prevent bit corruption
    GPIOA->MODER  &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->MODER  |=  ((2U << (0 * 2)) | (2U << (1 * 2)) | (2U << (6 * 2)) | (2U << (7 * 2))); // Alternate Function

    GPIOA->PUPDR  &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->PUPDR  |=  ((1U << (0 * 2)) | (1U << (1 * 2)) | (1U << (6 * 2)) | (1U << (7 * 2))); // Pull-up

    // AFR[0] (AFRL): PA0 (AF1), PA1 (AF1), PA6 (AF2), PA7 (AF2)
    GPIOA->AFR[0] &= ~((0xFU << (0 * 4)) | (0xFU << (1 * 4)) | (0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOA->AFR[0] |=  ((1U << (0 * 4))   | (1U << (1 * 4))   | (2U << (6 * 4))   | (2U << (7 * 4)));

    // 3. Configure TIM2 (Left Motor - 32 bit) in Encoder Mode 3
    TIM2->CR1   = 0;                    // Disable while configuring
    TIM2->PSC   = 0;                    // No prescaler (mandatory for encoder mode)
    TIM2->ARR   = 0xFFFFFFFF;
    TIM2->SMCR  = (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1); // SMS = 011: count both edges of TI1 and TI2
    TIM2->CCMR1 = (3U << 12) | (1U << 8) | (3U << 4) | (1U << 0); // CC1->TI1, CC2->TI2 + digital filter (ICxF=3)
    TIM2->CCER  = (TIM_CCER_CC1E | TIM_CCER_CC2E);               // Enable capture channels
    TIM2->CNT   = 0;
    TIM2->CR1   = TIM_CR1_CEN;          // Enable counter

    // 4. Configure TIM3 (Right Motor - 16 bit) in Encoder Mode 3
    TIM3->CR1   = 0;                    // Disable while configuring
    TIM3->PSC   = 0;                    // No prescaler (mandatory for encoder mode)
    TIM3->ARR   = 0xFFFF;
    TIM3->SMCR  = (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1); // SMS = 011: count both edges of TI1 and TI2
    TIM3->CCMR1 = (3U << 12) | (1U << 8) | (3U << 4) | (1U << 0); // CC1->TI1, CC2->TI2 + digital filter (ICxF=3)
    TIM3->CCER  = (TIM_CCER_CC1E | TIM_CCER_CC2E);               // Enable capture channels
    TIM3->CNT   = 0;
    TIM3->CR1   = TIM_CR1_CEN;          // Enable counter

    last_tim3   = 0;
    right_total = 0;
}

int32_t Encoder_GetLeft(void) {
    return (int32_t)-TIM2->CNT; // 32-bit hardware counter (inverted for forward robot convention)
}

int32_t Encoder_GetRight(void) {
    int16_t now = (int16_t)-TIM3->CNT;
    int16_t diff = (int16_t)(now - last_tim3); // Correctly handles 16-bit rollover
    last_tim3 = now;
    right_total += diff;
    return -right_total; // Inverted for forward robot convention
}

void Encoder_Read(int32_t *left, int32_t *right) {
    if (left)  *left  = Encoder_GetLeft();
    if (right) *right = Encoder_GetRight();
}

void Encoder_Reset(void) {
    TIM2->CNT   = 0;
    TIM3->CNT   = 0;
    last_tim3   = 0;
    right_total = 0;
}
