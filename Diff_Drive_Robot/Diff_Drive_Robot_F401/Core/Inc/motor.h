#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "stm32f4xx.h"

/**
 * TB6612FNG Dual DC Motor Driver Pin Configuration (GA25-370 Motors):
 *
 * PWM Pins (Timer 4):
 *   Left Motor  PWM (PWMA): PB6 (TIM4_CH1, AF2) -> Nucleo D10
 *   Right Motor PWM (PWMB): PB7 (TIM4_CH2, AF2) -> Nucleo CN7 pin 21
 *
 * Direction & Standby Control Pins (GPIOC):
 *   Left Motor  AIN1: PC0 -> Nucleo A5
 *   Left Motor  AIN2: PC1 -> Nucleo A4
 *   Right Motor BIN1: PC2 -> Nucleo A3
 *   Right Motor BIN2: PC3 -> Nucleo A2
 *   Standby     STBY: PC4 -> Nucleo CN10 pin 34 (Active HIGH)
 *
 * TB6612 Truth Table:
 *   IN1 = HIGH, IN2 = LOW  -> Forward
 *   IN1 = LOW,  IN2 = HIGH -> Reverse
 *   IN1 = LOW,  IN2 = LOW  -> Coast / Stop
 *   IN1 = HIGH, IN2 = HIGH -> Short Brake
 */

/* Maximum PWM duty cycle value (speed input range: -MOTOR_MAX_PWM to +MOTOR_MAX_PWM) */
#define MOTOR_MAX_PWM       1000

/* Tune these for the robot chassis and motors. */
#define MOTOR_TRACK_WIDTH_M         0.18f
#define MOTOR_MAX_WHEEL_SPEED_MPS   0.44f

/* Direction and Standby GPIO Definitions */
#define MOTOR_DIR_PORT      GPIOC
#define MOTOR_AIN1_PIN      0   /* PC0 */
#define MOTOR_AIN2_PIN      1   /* PC1 */
#define MOTOR_BIN1_PIN      2   /* PC2 */
#define MOTOR_BIN2_PIN      3   /* PC3 */
#define MOTOR_STBY_PIN      4   /* PC4 */

/* Function Prototypes */
void Motor_Init(void);
void Motor_SetCmdVel(float linear_x, float angular_z);
void Motor_SetSpeed(int16_t left, int16_t right);
void Motor_SetLeft(int16_t speed);
void Motor_SetRight(int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);
void Motor_Enable(void);
void Motor_Disable(void);

#endif /* INC_MOTOR_H_ */
