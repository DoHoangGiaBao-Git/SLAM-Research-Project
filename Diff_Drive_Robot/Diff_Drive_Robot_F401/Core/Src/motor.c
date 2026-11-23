#include "motor.h"

static int16_t velocity_to_pwm(float velocity) {
    float pwm = velocity * MOTOR_MAX_PWM / MOTOR_MAX_WHEEL_SPEED_MPS;

    if (pwm > MOTOR_MAX_PWM)  pwm = MOTOR_MAX_PWM;
    if (pwm < -MOTOR_MAX_PWM) pwm = -MOTOR_MAX_PWM;
    return (int16_t)pwm;
}

void Motor_SetCmdVel(float linear_x, float angular_z) {
    float half_track = MOTOR_TRACK_WIDTH_M / 2.0f;
    /* Keep positive ROS angular.z as a left turn on this robot. */
    int16_t left = velocity_to_pwm(linear_x + angular_z * half_track);
    int16_t right = velocity_to_pwm(linear_x - angular_z * half_track);

    Motor_SetSpeed(left, right);
}

void Motor_Init(void) {
    // 1. Enable peripheral clocks for GPIOB (PWM), GPIOC (Direction/STBY), and TIM4
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // 2. Configure GPIOC pins (PC0..PC4) as General Purpose Output Push-Pull
    GPIOC->MODER &= ~((3U << (MOTOR_AIN1_PIN * 2)) |
                      (3U << (MOTOR_AIN2_PIN * 2)) |
                      (3U << (MOTOR_BIN1_PIN * 2)) |
                      (3U << (MOTOR_BIN2_PIN * 2)) |
                      (3U << (MOTOR_STBY_PIN * 2)));
    GPIOC->MODER |=  ((1U << (MOTOR_AIN1_PIN * 2)) |
                      (1U << (MOTOR_AIN2_PIN * 2)) |
                      (1U << (MOTOR_BIN1_PIN * 2)) |
                      (1U << (MOTOR_BIN2_PIN * 2)) |
                      (1U << (MOTOR_STBY_PIN * 2)));

    GPIOC->OTYPER &= ~((1U << MOTOR_AIN1_PIN) |
                       (1U << MOTOR_AIN2_PIN) |
                       (1U << MOTOR_BIN1_PIN) |
                       (1U << MOTOR_BIN2_PIN) |
                       (1U << MOTOR_STBY_PIN));

    GPIOC->OSPEEDR |= ((1U << (MOTOR_AIN1_PIN * 2)) |
                       (1U << (MOTOR_AIN2_PIN * 2)) |
                       (1U << (MOTOR_BIN1_PIN * 2)) |
                       (1U << (MOTOR_BIN2_PIN * 2)) |
                       (1U << (MOTOR_STBY_PIN * 2)));

    GPIOC->PUPDR &= ~((3U << (MOTOR_AIN1_PIN * 2)) |
                      (3U << (MOTOR_AIN2_PIN * 2)) |
                      (3U << (MOTOR_BIN1_PIN * 2)) |
                      (3U << (MOTOR_BIN2_PIN * 2)) |
                      (3U << (MOTOR_STBY_PIN * 2)));

    // 3. Configure GPIOB pins (PB6: TIM4_CH1, PB7: TIM4_CH2) as Alternate Function AF2
    GPIOB->MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2))); // Alternate Function mode

    GPIOB->OSPEEDR |= ((2U << (6 * 2)) | (2U << (7 * 2))); // Fast speed
    GPIOB->PUPDR   &= ~((3U << (6 * 2)) | (3U << (7 * 2))); // No pull

    // AFR[0] (AFRL): PB6 -> AF2, PB7 -> AF2
    GPIOB->AFR[0] &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOB->AFR[0] |=  ((2U << (6 * 4))   | (2U << (7 * 4)));

    // 4. Configure TIM4 for PWM generation (21 kHz, ultrasonic & silent for GA25-370)
    // Clock = 84 MHz, PSC = 3 -> 84 MHz / 4 = 21 MHz
    // ARR = 999 -> Period = 1000 ticks -> 21 MHz / 1000 = 21 kHz
    TIM4->CR1   = 0;
    TIM4->PSC   = 3;
    TIM4->ARR   = MOTOR_MAX_PWM - 1;
    TIM4->CCR1  = 0;
    TIM4->CCR2  = 0;

    // PWM Mode 1 on CH1 and CH2 + Preload enable
    TIM4->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
                   TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);

    // Enable Channel 1 and Channel 2 outputs
    TIM4->CCER  = (TIM_CCER_CC1E | TIM_CCER_CC2E);

    // Auto-reload preload enable and start counter
    TIM4->CR1   = (TIM_CR1_ARPE | TIM_CR1_CEN);
    TIM4->EGR   = TIM_EGR_UG; // Generate update event to reload registers

    // 5. Enable driver (STBY = HIGH) and set initial motor state to stopped
    Motor_Enable();
    Motor_Stop();
}

void Motor_SetLeft(int16_t speed) {
    if (speed > MOTOR_MAX_PWM)  speed = MOTOR_MAX_PWM;
    if (speed < -MOTOR_MAX_PWM) speed = -MOTOR_MAX_PWM;

    if (speed > 0) {
        // Forward: AIN1 = 1, AIN2 = 0
        MOTOR_DIR_PORT->BSRR = (1U << MOTOR_AIN1_PIN) | (1U << (MOTOR_AIN2_PIN + 16));
        TIM4->CCR1 = (uint16_t)speed;
    } else if (speed < 0) {
        // Reverse: AIN1 = 0, AIN2 = 1
        MOTOR_DIR_PORT->BSRR = (1U << (MOTOR_AIN1_PIN + 16)) | (1U << MOTOR_AIN2_PIN);
        TIM4->CCR1 = (uint16_t)(-speed);
    } else {
        // Coast / Stop: AIN1 = 0, AIN2 = 0
        MOTOR_DIR_PORT->BSRR = (1U << (MOTOR_AIN1_PIN + 16)) | (1U << (MOTOR_AIN2_PIN + 16));
        TIM4->CCR1 = 0;
    }
}

void Motor_SetRight(int16_t speed) {
    if (speed > MOTOR_MAX_PWM)  speed = MOTOR_MAX_PWM;
    if (speed < -MOTOR_MAX_PWM) speed = -MOTOR_MAX_PWM;

    if (speed > 0) {
        // Forward: BIN1 = 1, BIN2 = 0
        MOTOR_DIR_PORT->BSRR = (1U << MOTOR_BIN1_PIN) | (1U << (MOTOR_BIN2_PIN + 16));
        TIM4->CCR2 = (uint16_t)speed;
    } else if (speed < 0) {
        // Reverse: BIN1 = 0, BIN2 = 1
        MOTOR_DIR_PORT->BSRR = (1U << (MOTOR_BIN1_PIN + 16)) | (1U << MOTOR_BIN2_PIN);
        TIM4->CCR2 = (uint16_t)(-speed);
    } else {
        // Coast / Stop: BIN1 = 0, BIN2 = 0
        MOTOR_DIR_PORT->BSRR = (1U << (MOTOR_BIN1_PIN + 16)) | (1U << (MOTOR_BIN2_PIN + 16));
        TIM4->CCR2 = 0;
    }
}

void Motor_SetSpeed(int16_t left, int16_t right) {
    Motor_SetLeft(left);
    Motor_SetRight(right);
}

void Motor_Stop(void) {
    Motor_SetSpeed(0, 0);
}

void Motor_Brake(void) {
    // Active brake: both inputs HIGH, PWM = 0
    MOTOR_DIR_PORT->BSRR = (1U << MOTOR_AIN1_PIN) | (1U << MOTOR_AIN2_PIN) |
                           (1U << MOTOR_BIN1_PIN) | (1U << MOTOR_BIN2_PIN);
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
}

void Motor_Enable(void) {
    MOTOR_DIR_PORT->BSRR = (1U << MOTOR_STBY_PIN);
}

void Motor_Disable(void) {
    MOTOR_DIR_PORT->BSRR = (1U << (MOTOR_STBY_PIN + 16));
}
