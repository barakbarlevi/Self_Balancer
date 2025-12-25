#include "PID.h"
#include "motors.h"

/**
 * @brief Initializes a PID controller structure with given gains and type.
 * @param pid Pointer to the PID structure to initialize.
 * @param type The type of PID controller: pitch angle, speed, or yaw angle.
 */
void PID_Init(PID_t *pid, PID_Type_t type, float Kp, float Ki, float Kd) {
	pid->type = type;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;

    switch (type) {

    case PID_TYPE_PITCH_ANGLE:
    	pid->output_limit = MAX_EFFECTIVE_PWM;
    	pid->raw_integral_limit = pid->output_limit / Ki / 2;
    	break;

    case PID_TYPE_SPEED:
		pid->output_limit = MAX_EFFECTIVE_PWM * 0.3;
	    pid->raw_integral_limit = pid->output_limit / Ki;
	    break;

    case PID_TYPE_YAW_ANGLE:
		pid->output_limit = MAX_EFFECTIVE_PWM;
		pid->raw_integral_limit = pid->output_limit / Ki;
		break;
    }

}


/**
 * @brief Updates the PID controller output based on current measurement.
 * @param pid:  Pointer to the PID structure.
 * @param target:  The desired setpoint.
 * @param measurement:  The current measured value.
 * @param measured_rate:  The measured rate (used for the derivative term in the angle controllers).
 * @param dt:  Time elapsed since the last update, in seconds.
 * @retval float:  The computed PID output, limited to the configured output range.
 */
float PID_Update(PID_t *pid, float target, float measurement, float measured_rate, float dt) {

	if (!isfinite(target) || !isfinite(measurement) ||
	    !isfinite(measured_rate) || !isfinite(dt) || dt <= 0.0f) {
	    return pid->output; // fail-safe hold
	}

	float error = measurement - target;
	float derivative;

	switch (pid->type)
	{
		case PID_TYPE_PITCH_ANGLE:

			error = measurement - target;
			derivative = measured_rate;
			break;

		case PID_TYPE_YAW_ANGLE:
			error = measurement - target;
			derivative = measured_rate;
			break;

		case PID_TYPE_SPEED:
			error = target - measurement;
			derivative = (error - pid->prev_error) / dt;

		default:
			derivative = (error - pid->prev_error) / dt;
			break;
	}

    pid->integral += error * dt;

    // Anti-windup clamp on the raw integral
    if (pid->integral > pid->raw_integral_limit)
        pid->integral = pid->raw_integral_limit;

    if (pid->integral < -pid->raw_integral_limit)
        pid->integral = -pid->raw_integral_limit;


    pid->prev_error = error;

    // Compute, limit, store and return the output
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    if (output > pid->output_limit) output = pid->output_limit;
	if (output < -pid->output_limit) output = -pid->output_limit;
	pid->output = output;

	if (!isfinite(output)) {
	    output = 0.0f;
	    pid->integral = 0.0f;
	}
	
	return output;
}
