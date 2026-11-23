#ifndef INC_ODOM_H_
#define INC_ODOM_H_

#include <stdint.h>

/* Tune these for the robot. TICKS_PER_REV includes the encoder's x4 counting. */
#define ODOM_WHEEL_RADIUS_M  0.0325f
#define ODOM_TRACK_WIDTH_M   0.18f
#define ODOM_TICKS_PER_REV   1975.0f

typedef struct {
    int32_t last_left_ticks;
    int32_t last_right_ticks;
    float x;
    float y;
    float yaw;
    float linear_velocity;
    float angular_velocity;
} OdomState;

void Odom_Init(OdomState *odom, int32_t left_ticks, int32_t right_ticks);
void Odom_Update(OdomState *odom, int32_t left_ticks, int32_t right_ticks, float dt);

#endif /* INC_ODOM_H_ */
