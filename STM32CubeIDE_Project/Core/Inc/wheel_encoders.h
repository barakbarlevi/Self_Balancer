/*
 * wheel_encoders.h
 *
 *  Created on: Nov 5, 2025
 *      Author: barak
 */

#ifndef INC_WHEEL_ENCODERS_H_
#define INC_WHEEL_ENCODERS_H_

#include "stm32f3xx_hal.h"
#include <stdio.h>

#define ENCODER_LEFT_ADDR    (0x40) // << 1)
#define ENCODER_RIGHT_ADDR   (0x41) // << 1)
#define ENCODER_REG_START    0xFE

HAL_StatusTypeDef singleEncReading(uint16_t encoderAddr, float *angle_out);
void readShaftPositions(float *angleLeft, float *angleRight);
void I2C_Scan(void);



#endif /* INC_WHEEL_ENCODERS_H_ */
