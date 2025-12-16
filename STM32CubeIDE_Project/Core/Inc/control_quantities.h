/**
 * Defines the structure and functions for storing and managing
 * the control quantities for the robot. These control quantities are independent of
 * the control type.
 */

#ifndef INC_CONTROL_QUANTITIES_H_
#define INC_CONTROL_QUANTITIES_H_

#include <stdint.h>

#define CONTROL_PERIOD_MS 5.0

typedef struct {
    float target_pitch_angle;
    float target_yaw_angle;
    float target_speed;

    float u_pitch_angle;
    float u_yaw_angle_hold;
    float u_speed_hold;

    float desired_control_period_ms;
    float dt_actual;

    float dt_chassis;
    uint32_t last_chassis_tick;
} ControlQuantities_t;

void Control_Quantites_Init(ControlQuantities_t *ControlQuantities);

#endif /* INC_CONTROL_QUANTITIES_H_ */
