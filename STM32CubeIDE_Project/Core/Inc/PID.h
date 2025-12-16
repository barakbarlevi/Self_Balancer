/**
 * Implementation of PID controllers for pitch, yaw, and speed control.
 */
#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm_pi_uart1_link.h"

typedef enum {
    PID_TYPE_PITCH_ANGLE,
    PID_TYPE_SPEED,
	PID_TYPE_YAW_ANGLE
} PID_Type_t;

typedef struct {
	PID_Type_t type;
	float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float raw_integral_limit;  // Raw integral, as opposed to pid->Ki * pid->integral
    float output;
	float output_limit;
} PID_t;

void PID_Init(PID_t *pid, PID_Type_t type, float Kp, float Ki, float Kd);
float PID_Update(PID_t *pid, float target, float measurement, float measured_rate, float dt);

#endif /* INC_PID_H_ */
