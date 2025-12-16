#include "operator_commands.h"
#include <string.h>

void Operator_Commands_Init(operator_commands_t* operator_commands) {
	memset(operator_commands, 0, sizeof(*operator_commands));
}

/**
 * @brief Compute a temporary speed setpoint named "Route 1" with ramp-up and ramp-down.
 * This function generates a time-dependent target speed for a predefined route.
 * It supports an initial ramp-up period, a constant-speed phase, and a ramp-down period. The ramps' purpose is
 * to ease setpoint changes that may be too abrupt for the control.
 * At the end of the route, the speed is set to zero and relevant operator command flags are reset.
 *
 * @param now_ms:  Current main control loop time
 * @param total_duration_ms:  Total desired duration of the maneuver
 * @param ramp_up_ms:  Duration of the speed ramp-up phase until cruising at operator_commands.temp_target_speed
 * @param ramp_down_ms:  Duration of the speed ramp-down phase
 *
 * @retval target_speed:  Computed target speed for the current time step to be set as PWM command
 */
float do_route1(uint32_t now_ms,uint32_t total_duration_ms, uint32_t ramp_up_ms, uint32_t ramp_down_ms)
{
	float target_speed;

    /* ------------ FIRST ACTIVATION ------------ */
    if (!operator_commands.speed_ramp_active) {
    	operator_commands.speed_start_time = now_ms;
    	operator_commands.speed_ramp_active = 1;
        return 0.0f;
    }

    uint32_t elapsed_ms = now_ms - operator_commands.speed_start_time;

    /* ------------ RAMP FINISHED ------------ */
    if (elapsed_ms >= total_duration_ms) {
        target_speed = 0.0f;
        operator_commands.speed_ramp_active = 0;
        stm_pi_uart1_link.route1_activate = 0;
        operator_commands.motorTestsIntervalCounter++;
        operator_commands.motorTestsDoOnceDummy = 0;
        return 0.0f;
    }

    /* ------------ RAMP SHAPING ------------ */
    if (elapsed_ms < ramp_up_ms) {
        target_speed = operator_commands.temp_target_speed * ((float)elapsed_ms / ramp_up_ms);
        return target_speed;
    }
    else if (elapsed_ms > (total_duration_ms - ramp_down_ms) ) {
        target_speed = operator_commands.temp_target_speed *
                       (1.0f - (float)(elapsed_ms - (total_duration_ms - ramp_down_ms) ) / ramp_down_ms);
        return target_speed;
    }
    else {
        target_speed = operator_commands.temp_target_speed;
        return target_speed;
    }
}


/**
 * @brief See do_route1() description. do_route2() is just another possible operator command. do_route2() changes the
 * momentary yaw command instead of speed
 */
float do_route2(uint32_t now_ms,uint32_t total_duration_ms, uint32_t ramp_up_ms, uint32_t ramp_down_ms)
{
	float target_yaw_command;

    /* ------------ FIRST ACTIVATION ------------ */
    if (!operator_commands.yaw_ramp_active) {
    	operator_commands.yaw_start_time = now_ms;
    	operator_commands.yaw_ramp_active = 1;
        return 0.0f;
    }

    uint32_t elapsed_ms = now_ms - operator_commands.yaw_start_time;

    /* ------------ RAMP FINISHED ------------ */
    if (elapsed_ms >= total_duration_ms) {
    	target_yaw_command = 0.0f;
    	operator_commands.yaw_ramp_active = 0;
        stm_pi_uart1_link.route2_activate = 0;
        operator_commands.motorTestsIntervalCounter++;
        operator_commands.motorTestsDoOnceDummy = 0;
        return 0.0f;
    }

    /* ------------ RAMP SHAPING ------------ */
    if (elapsed_ms < ramp_up_ms) {
    	target_yaw_command = operator_commands.temp_target_yaw_command * ((float)elapsed_ms / ramp_up_ms);
        return target_yaw_command;
    }
    else if (elapsed_ms > (total_duration_ms - ramp_down_ms) ) {
    	target_yaw_command = operator_commands.temp_target_yaw_command *
                       (1.0f - (float)(elapsed_ms - (total_duration_ms - ramp_down_ms) ) / ramp_down_ms);
        return target_yaw_command;
    }
    else {
    	target_yaw_command = operator_commands.temp_target_yaw_command;
        return target_yaw_command;
    }
}


/**
 * @brief See do_route1() description. do_turnWheel() is just another possible operator command. do_turnWheel() changes the
 * PWM straightly. It's intended to be used when performing wheel rotation trials rather than balance control.
 */
float do_turnWheel(uint32_t now_ms,uint32_t total_duration_ms, uint32_t ramp_up_ms, uint32_t ramp_down_ms)
{
	float target_PWM;

    /* ------------ FIRST ACTIVATION ------------ */
    if (!operator_commands.motor_trial_ramp_active) {
    	operator_commands.motor_trial_start_time = now_ms;
    	operator_commands.motor_trial_ramp_active = 1;
        return 0.0f;
    }

    uint32_t elapsed_ms = now_ms - operator_commands.motor_trial_start_time;

    /* ------------ RAMP FINISHED ------------ */
    if (elapsed_ms >= total_duration_ms) {
    	target_PWM = 0.0f;
        operator_commands.motor_trial_ramp_active = 0;
        stm_pi_uart1_link.route1_activate = 0; // TODO: Have this run on a command of its own instead of route1_activate...
        operator_commands.motorTestsIntervalCounter++;
        operator_commands.motorTestsDoOnceDummy = 0;
        return 0.0f;
    }

    /* ------------ RAMP SHAPING ------------ */
    if (elapsed_ms < ramp_up_ms) {
    	target_PWM = operator_commands.temp_target_PWM * ((float)elapsed_ms / ramp_up_ms);
        return target_PWM;
    }
    else if (elapsed_ms > (total_duration_ms - ramp_down_ms) ) {
    	target_PWM = operator_commands.temp_target_PWM *
                       (1.0f - (float)(elapsed_ms - (total_duration_ms - ramp_down_ms) ) / ramp_down_ms);
        return target_PWM;
    }
    else {
    	target_PWM = operator_commands.temp_target_PWM;
        return target_PWM;
    }
}

