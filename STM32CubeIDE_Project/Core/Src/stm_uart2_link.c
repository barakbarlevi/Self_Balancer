#include "stm32f3xx_hal.h"
#include "stm_uart2_link.h"


/**
 * @brief  Packs a 32-bit floating-point value into a byte buffer in little-endian order.
 * @param  dest: Pointer to a buffer where the 4-byte little-endian float will be written.
 * The buffer must have space for at least 4 bytes.
 * @param  value: Floating-point value to pack.
 */
static inline void pack_float_le(uint8_t *dest, float value)
{
    union { float f; uint8_t b[4]; } u;
    u.f = value;
    dest[0] = u.b[0];
    dest[1] = u.b[1];
    dest[2] = u.b[2];
    dest[3] = u.b[3];
}


/**
  * @brief  Packs telemetry data into a frame and sends it over UART using ST-Link.
  * @retval None
  * @note   HAL_UART_Transmit_IT is used for non-blocking transmission; the function
  *         will return immediately if UART is busy, dropping the frame.
  */
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
						  float dt) {

	static uint8_t pitch_pid_info_frame[STM_TO_UART2_TELEM_FRAME_TOTAL_SIZE];

    /* ---- drop frame if UART still busy ---- */
    if (huart->gState != HAL_UART_STATE_READY) {
        return;
    }

	// write header (little endian)
	pitch_pid_info_frame[0] = (uint8_t)STM_TO_UART2_TELEM_FRAME_HEADER_1;
	pitch_pid_info_frame[1] = (uint8_t)STM_TO_UART2_TELEM_FRAME_HEADER_2;

	pack_float_le(&pitch_pid_info_frame[2],  pitch_kalman1x1_estimate);
	pack_float_le(&pitch_pid_info_frame[6],  imu_gx);
	pack_float_le(&pitch_pid_info_frame[10], u_pitch_angle);
	pack_float_le(&pitch_pid_info_frame[14], chassisData_xdot);
	pack_float_le(&pitch_pid_info_frame[18], chassisData_left_wheel_speed_rad_sec);
	pack_float_le(&pitch_pid_info_frame[22], chassisData_right_wheel_speed_rad_sec);
	pack_float_le(&pitch_pid_info_frame[26], u_speed_hold);
	pack_float_le(&pitch_pid_info_frame[30], (float)pwm_left);
	pack_float_le(&pitch_pid_info_frame[34], (float)pwm_right);
	pack_float_le(&pitch_pid_info_frame[38], u_forward);
	pack_float_le(&pitch_pid_info_frame[42], chassis_psidot);
	pack_float_le(&pitch_pid_info_frame[46], chassis_yaw_angle);
	pack_float_le(&pitch_pid_info_frame[50], yaw_kalman1x1_estimate);
	pack_float_le(&pitch_pid_info_frame[54], imu_gz);
	pack_float_le(&pitch_pid_info_frame[58], yaw_angle_kalman_rate_integration);
	pack_float_le(&pitch_pid_info_frame[62], imu_gy);
	pack_float_le(&pitch_pid_info_frame[66], yaw_rate_world);
	pack_float_le(&pitch_pid_info_frame[70], yaw_angle_IMU_rate_integ_no_kalman);
	pack_float_le(&pitch_pid_info_frame[74], speed_pid_integral);
	pack_float_le(&pitch_pid_info_frame[78], u_turn);
	pack_float_le(&pitch_pid_info_frame[82], (float)pid_update_available);
	pack_float_le(&pitch_pid_info_frame[86], kp_angle);
	pack_float_le(&pitch_pid_info_frame[90], ki_angle);
	pack_float_le(&pitch_pid_info_frame[94], kd_angle);
	pack_float_le(&pitch_pid_info_frame[98], kp_speed);
	pack_float_le(&pitch_pid_info_frame[102], ki_speed);
	pack_float_le(&pitch_pid_info_frame[106], kd_speed);
	pack_float_le(&pitch_pid_info_frame[110], kp_yaw);
	pack_float_le(&pitch_pid_info_frame[114], ki_yaw);
	pack_float_le(&pitch_pid_info_frame[118], dt);

	//HAL_UART_Transmit(huart, pitch_pid_info_frame, sizeof(pitch_pid_info_frame), HAL_MAX_DELAY);
	HAL_UART_Transmit_IT(huart, pitch_pid_info_frame, sizeof(pitch_pid_info_frame));
}
