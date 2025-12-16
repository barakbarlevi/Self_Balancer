/*
 * Operator commands may be for example setting temporary speed / yaw setpoints ("routes"),
 * commanding wheel rotations for tests,
 * zeroing PID accumulated integrals, or any other functionality required.
 *
 * Host PC --> SSH --> On-board main computer (Raspberry Pi) command line
 * --> echo a command on a local fifo --> Send to STM --> STM recognizes and takes action
 */

#ifndef INC_OPERATOR_COMMANDS_H_
#define INC_OPERATOR_COMMANDS_H_

#include <stdint.h>
#include "stm_pi_uart1_link.h"

#define ROUTE1_FRAME_HEADER_1 0xA6
#define ROUTE1_FRAME_HEADER_2 0xC6
#define ROUTE2_FRAME_HEADER_1 0xB6
#define ROUTE2_FRAME_HEADER_2 0xD6

typedef struct
{
	uint32_t speed_start_time;
	uint8_t speed_ramp_active;
	float temp_target_speed;

	uint32_t yaw_start_time;
	uint8_t yaw_ramp_active;
	float temp_target_yaw_command;


	uint32_t motor_trial_start_time;
	uint8_t motor_trial_ramp_active;
	int temp_target_PWM;
	int motorTestsIntervalCounter; // This counter is used when in motor test mode. It counts up to the number of test intervals
	int motorTestsDoOnceDummy;     // This dummy variable is used when in motor test mode


} operator_commands_t;

extern operator_commands_t operator_commands;

void Operator_Commands_Init(operator_commands_t* operator_commands);
float do_route1(uint32_t now_ms,uint32_t total_duration_ms, uint32_t ramp_up_ms, uint32_t ramp_down_ms);
float do_route2(uint32_t now_ms,uint32_t total_duration_ms, uint32_t ramp_up_ms, uint32_t ramp_down_ms);
float do_turnWheel(uint32_t now_ms,uint32_t total_duration_ms, uint32_t ramp_up_ms, uint32_t ramp_down_ms);


#endif /* INC_OPERATOR_COMMANDS_H_ */

