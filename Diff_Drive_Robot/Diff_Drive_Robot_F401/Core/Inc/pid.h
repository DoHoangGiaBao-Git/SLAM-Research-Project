#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

/* Tune these gains for the motors. Output is PWM counts. */
#define PID_KP                 1.0f
#define PID_KI                  0.0f
#define PID_KD                   0.0f
#define PID_INTEGRAL_LIMIT       1.0f

typedef struct {
    float integral;
    float previous_speed;
} WheelPID;

typedef struct {
    WheelPID left;
    WheelPID right;
    int32_t previous_left_ticks;
    int32_t previous_right_ticks;
} PIDController;

void PID_Init(PIDController *pid, int32_t left_ticks, int32_t right_ticks);

/* cmd_vel speeds are m/s and rad/s; dt is seconds. */
void PID_Update(PIDController *pid,
                float linear_speed,
                float angular_speed,
                int32_t left_ticks,
                int32_t right_ticks,
                float dt);

#endif /* INC_PID_H_ */
