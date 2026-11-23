#include "odom.h"
#include <math.h>

#define ODOM_PI       3.14159265f
#define ODOM_TWO_PI   (2.0f * ODOM_PI)

void Odom_Init(OdomState *odom, int32_t left_ticks, int32_t right_ticks) {
    odom->last_left_ticks = left_ticks;
    odom->last_right_ticks = right_ticks;
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->yaw = 0.0f;
    odom->linear_velocity = 0.0f;
    odom->angular_velocity = 0.0f;
}

void Odom_Update(OdomState *odom, int32_t left_ticks, int32_t right_ticks, float dt) {
    if (dt <= 0.0f) return;

    float meters_per_tick = ODOM_TWO_PI * ODOM_WHEEL_RADIUS_M / ODOM_TICKS_PER_REV;
    float left_distance = (float)((int64_t)left_ticks - odom->last_left_ticks) * meters_per_tick;
    float right_distance = (float)((int64_t)right_ticks - odom->last_right_ticks) * meters_per_tick;
    float distance = (left_distance + right_distance) / 2.0f;
    float angle = (right_distance - left_distance) / ODOM_TRACK_WIDTH_M;
    float midpoint_yaw = odom->yaw + angle / 2.0f;

    odom->x += distance * cosf(midpoint_yaw);
    odom->y += distance * sinf(midpoint_yaw);
    odom->yaw += angle;

    if (odom->yaw > ODOM_PI) odom->yaw -= ODOM_TWO_PI;
    if (odom->yaw < -ODOM_PI) odom->yaw += ODOM_TWO_PI;

    odom->linear_velocity = distance / dt;
    odom->angular_velocity = angle / dt;
    odom->last_left_ticks = left_ticks;
    odom->last_right_ticks = right_ticks;
}