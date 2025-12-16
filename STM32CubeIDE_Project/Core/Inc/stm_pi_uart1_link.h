/**
 * UART1 communication link between STM32 and Raspberry Pi
 * Communication over this peripheral is made using pin initialization and 2 jumper wires.
 *
 * The link passes in this current version:
 * 1. Chassis data from the pi to the STM
 *
 * 2. Tuning data, mainly used for PID gain tuning but not strictly. The pi is receiving these
 * values over WiFi from a station (laptop) that gets potentiometer readings from 8 analog i/o pins on an arduino utilizing its 8-bit ADC.
 *
 * 3. Operator commands, see operator_commands.h
 *
 * 4. A ready_byte from the STM to the pi, signaling it's ready to receive and parse a packet.
 *
 * This module defines the data structures and constants needed for communication over UART1
 * between the on-board STM32 MCU and the on-board Raspberry Pi. It handles fixed-size frames to simplify parsing.
 */
#ifndef INC_STM_PI_UART1_LINK_H_
#define INC_STM_PI_UART1_LINK_H_

#include <stdint.h>

#define PI_TO_STM_FRAME_TOTAL_SIZE   35  // 2 (header) + 4*8 + 1 crc byte
										 // 8 floats in order to make the message length from the Pi a fixed size, and is determined
										 // by the message with the max length, which is the PID tuning parameters. So in the case
										 // where a chassis frame is passed, 4 floats are 0 paddings
#define PID_TUNING_FRAME_HEADER_1  0xA9
#define PID_TUNING_FRAME_HEADER_2  0xC9
#define CHASSIS_FRAME_HEADER_1     0x44
#define CHASSIS_FRAME_HEADER_2     0xBB
#define READY_BYTE 				   0xD4



typedef struct
{
	uint8_t ready_byte;									// STM-->pi. Signal it's ready to receive and parse a packet.
	uint8_t rx_buffer[PI_TO_STM_FRAME_TOTAL_SIZE];		// Receive buffer. Fixed size to all types of data coming from the pi (determined by the largest packet)
	uint8_t pid_buffer[PI_TO_STM_FRAME_TOTAL_SIZE];		// If PID tuning packet, store in this buffer
	uint8_t chassis_buffer[PI_TO_STM_FRAME_TOTAL_SIZE]; // If chassis kinematics data, store in this buffer
	uint8_t chassis_update_available;					// Flag is set after a chassis kinematics packet is done being processed
	uint8_t route1_activate;							// Activation flag for an operator command designated "route1"
	uint8_t route2_activate;							// Activation flag for an operator command designated "route2"
	int pid_update_available;							// Flag is set after a tuning packet is done being processed

} stm_pi_uart1_link_t;

void STM_Pi_UART1_Link_Init(stm_pi_uart1_link_t *stm_pi_uart1_link);

extern stm_pi_uart1_link_t stm_pi_uart1_link;

#endif /* INC_STM_PI_UART1_LINK_H_ */
