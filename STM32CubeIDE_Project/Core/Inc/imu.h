/*
 * Provides definitions, data structures, and functions for interfacing with the IMU over SPI.
 * IMU in use is https://www.mouser.co.il/ProductDetail/81-SCH16T-K10-PCB
 * Datasheet: https://sensorsandpower.angst-pfister.com/fileadmin/products/datasheets/191/SCH16T-K01_1640-21648-0034-E-0125.pdf

 * Reading gyroscope and accelerometers data on channels *_XYZ1. 48-bit operations mode, polling mode.
 * These channels (*_XYZ1) implement internal interpolation. The IMU's raw measurements default frequency
 * is 11.8[kHz] which is every 84.7[usec]. Linear interpolation uses 1 future raw sample and updates an output register at 377.6[kHz],
 * which is every 2.65[usec], with a linearly interpolated result. This provides a more accurate estimate of the sensor value at the
 * time the read request was made, and reduces sampling time uncertainty from 84.7[usec] to 2.65[usec].
 *
 *
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f3xx_hal.h"
#include <stdio.h>

#define IMU_NSS_GPIO_Port GPIOA
#define IMU_NSS_Pin       GPIO_PIN_4
#define STATUS_REG_COUNT 12
#define MAX_STATUS_RETRIES 3
#define ACC_SENSITIVITY 3200.0f
#define GYRO_SENSITIVITY 1600.0f
#define IMU_DATA_FRAME_HEADER 0xAA55
#define ACC_MAX 20.0f       // m/s²
#define GYRO_MAX 2000.0f    // deg/s

typedef struct {
	float ax, ay, az;
	float gx, gy, gz;
} IMU_Data_t;

void IMU_Select();
void IMU_Deselect();
void IMU_StartUpSequence(SPI_HandleTypeDef *hspi);
void IMU_ReadSerialNumber(SPI_HandleTypeDef *hspi);
int32_t sign_extend_20bit(uint32_t raw);
void send_IMU_frame(UART_HandleTypeDef *huart, IMU_Data_t *imu);
void single_IMU_reading(SPI_HandleTypeDef *hspi,  IMU_Data_t *imu);




#endif /* INC_IMU_H_ */
