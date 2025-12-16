/**
 * UART2 communication link between STM32 and Raspberry Pi
 * Communication over this peripheral is done with a usb cable, which also powers the STM32 from the pi.
 * The STM32’s UART2 peripheral is connected to the ST-Link virtual COM port, allowing USB-based
 * serial communication with a host PC or Raspberry Pi.
 *
 * The link passes in this current version:
 * 1. Live telemetry data from the STM to the pi/Host PC, using the function send_telem_frame_over_STLink().
 */

#ifndef INC_STM_UART2_LINK_H_
#define INC_STM_UART2_LINK_H_

#define STM_TO_UART2_TELEM_FRAME_HEADER_1 0x88
#define STM_TO_UART2_TELEM_FRAME_HEADER_2 0xEF
#define STM_TO_UART2_TELEM_FRAME_PAYLOAD_SIZE 120   // 30 floats of 4 bytes each
#define STM_TO_UART2_TELEM_FRAME_TOTAL_SIZE (2 + STM_TO_UART2_TELEM_FRAME_PAYLOAD_SIZE)

void send_telem_frame_over_STLink(UART_HandleTypeDef *huart,
		                  float pitch_kalman1x1_estimate,
						  float imu_gx,
						  float u_pitch_angle,
						  float chassisData_xdot,
						  float chassisData_left_wheel_speed_rad_sec,
						  float chassisData_right_wheel_speed_rad_sec,
						  float u_speed_hold,
						  int pwm_left,
						  int pwm_right,
						  float u_forward,
						  float chassis_psidot,
						  float chassis_yaw_angle,
						  float yaw_kalman1x1_estimate,
						  float imu_gz,
						  float yaw_angle_kalman_rate_integration,
						  float imu_gy,
						  float yaw_rate_world,
						  float yaw_angle_IMU_rate_integ_no_kalman,
						  float speed_pid_integral,
						  float u_turn,
						  int pid_update_available,
						  float kp_angle,
						  float ki_angle,
						  float kd_angle,
						  float kp_speed,
						  float ki_speed,
						  float kd_speed,
						  float kp_yaw,
						  float ki_yaw,
						  float dt);

#endif /* INC_STM_UART2_LINK_H_ */
