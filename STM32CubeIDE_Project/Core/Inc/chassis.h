/**
 * Wheel encoders (https://docs.rs-online.com/067b/A700000006921293.pdf) are placed in both wheel assemblies.
 * The on-board raspberry pi reads their data over I2C. It then uses simple matrix transformations to convert
 * the [rad/sec] measurements to chassis speed (forward is positive) and psi_dot (yaw angle change rate, positive
 * in the CCW direction when looking from above). This is done in the kinematics script on the pi.
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include "stm32f3xx_hal.h"
#include "stm_pi_uart1_link.h"

typedef struct {
	float left_wheel_speed_rad_sec;
    float right_wheel_speed_rad_sec;
    float xdot;
    float psidot;
    float yaw_angle;
    int dt_chassis_first_calc_flag;
} ChassisData_t;

void ChassisData_Init(ChassisData_t* chassisData);

#endif /* INC_CHASSIS_H_ */
