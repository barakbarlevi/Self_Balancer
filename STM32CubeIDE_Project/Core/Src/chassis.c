#include <stdint.h>
#include "chassis.h"

void ChassisData_Init(ChassisData_t* chassisData) {
	chassisData->left_wheel_speed_rad_sec = 0.0f;
	chassisData->psidot = 0.0f;
	chassisData->right_wheel_speed_rad_sec = 0.0f;
	chassisData->xdot = 0.0f;
	chassisData->yaw_angle = 0.0f;
	chassisData->dt_chassis_first_calc_flag = 0;
}
