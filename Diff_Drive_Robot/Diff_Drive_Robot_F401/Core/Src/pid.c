#include "pid.h"
#include "motor.h"
#include "odom.h"

static float limit(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void PID_Init(PIDController *pid, int32_t left_ticks, int32_t right_ticks) {
    if (pid == 0) return;

    pid->left.integral = 0.0f;
    pid->left.previous_speed = 0.0f;
    pid->right.integral = 0.0f;
    pid->right.previous_speed = 0.0f;
    pid->previous_left_ticks = left_ticks;
    pid->previous_right_ticks = right_ticks;
}

static int16_t wheel_pid(WheelPID *wheel, float target_speed,
                         float measured_speed, float dt) {
    float error = target_speed - measured_speed;
    float acceleration = (measured_speed - wheel->previous_speed) / dt;
    float feed_forward = target_speed *
                         ((float)MOTOR_MAX_PWM / MOTOR_MAX_WHEEL_SPEED_MPS);
    float pwm;

    wheel->integral = limit(wheel->integral + error * dt,
                            -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    wheel->previous_speed = measured_speed;

    /* Feed-forward gives a starting PWM; PID corrects the speed error. */
    pwm = feed_forward + PID_KP * error + PID_KI * wheel->integral -
          PID_KD * acceleration;
    return (int16_t)limit(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
}

void PID_Update(PIDController *pid,
                float linear_speed,
                float angular_speed,
                int32_t left_ticks,
                int32_t right_ticks,
                float dt) {
    const float meters_per_tick =
        (6.2831853f * ODOM_WHEEL_RADIUS_M) / ODOM_TICKS_PER_REV;
    int64_t left_delta;
    int64_t right_delta;
    float left_target;
    float right_target;
    float left_speed;
    float right_speed;

    if (pid == 0 || dt <= 0.0f) return;

    left_delta = (int64_t)left_ticks - pid->previous_left_ticks;
    right_delta = (int64_t)right_ticks - pid->previous_right_ticks;
    pid->previous_left_ticks = left_ticks;
    pid->previous_right_ticks = right_ticks;

    left_speed = (float)left_delta * meters_per_tick / dt;
    right_speed = (float)right_delta * meters_per_tick / dt;

    /* This robot's motor sides are reversed relative to the usual convention. */
    left_target = linear_speed + angular_speed * MOTOR_TRACK_WIDTH_M / 2.0f;
    right_target = linear_speed - angular_speed * MOTOR_TRACK_WIDTH_M / 2.0f;
    left_target = limit(left_target, -MOTOR_MAX_WHEEL_SPEED_MPS,
                        MOTOR_MAX_WHEEL_SPEED_MPS);
    right_target = limit(right_target, -MOTOR_MAX_WHEEL_SPEED_MPS,
                         MOTOR_MAX_WHEEL_SPEED_MPS);

    Motor_SetSpeed(wheel_pid(&pid->left, left_target, left_speed, dt),
                   wheel_pid(&pid->right, right_target, right_speed, dt));
}
