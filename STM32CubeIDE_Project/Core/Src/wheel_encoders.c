/*
 * wheel_encoders.c
 *
 *  Created on: Nov 5, 2025
 *      Author: barak
 */


#include "wheel_encoders.h"

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef singleEncReading(uint16_t encoderAddr, float *angle_out)
{
//    uint8_t reg = ENCODER_REG_START;
//    uint8_t reg = 0xFE;
    uint8_t buffer[2];
//    HAL_StatusTypeDef ret;

	if (HAL_I2C_Mem_Read(&hi2c1, encoderAddr << 1, 0xFE, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK)
	{
		printf("HAL_I2C_Mem_Read Error: 0x%lX\r\n", HAL_I2C_GetError(&hi2c1));
		return HAL_ERROR;
	}
	printf("Encoder bytes: 0x%02X 0x%02X\r\n", buffer[0], buffer[1]);
	uint16_t binaryPosition = ((uint16_t)buffer[0] << 6) | (buffer[1]);
//	uint16_t binaryPosition = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x3FFF;
//	uint16_t binaryPosition = ((uint16_t)buffer[0] << 6) | (buffer[1] & 0x3F);
//	uint16_t binaryPosition = ((uint16_t)buffer[1] << 6) | (buffer[0] & 0x3F);

//	uint16_t raw = ((uint16_t)buffer[0] << 8) | buffer[1];
//	uint16_t binaryPosition = (raw >> 2) & 0x3FFF;

    // Convert to degrees: binary * (360 / 2^14)
    //float degreesPosition = (float)binaryPosition * (360.0f / 16384.0f);  // This is twice the actual displacement, zeros at 360 <=> actual 180
	float degreesPosition = (float)binaryPosition * (360.0f / 32768.0f);    // This is the actual displacement, zeros at 180 <=> actual 180

    // Round to nearest 0.1
    // multiply by 10, add 0.5 for rounding, truncate, divide by 10
    float temp = degreesPosition * 10.0f;
    int tmpInt = (int)(temp + 0.5f); // works for non-negative values (angles are 0..360)
    *angle_out = (float)tmpInt / 10.0f;

    return HAL_OK;
}


// ---------------------------------------------------
// Function: readShaftPositions()
// Reads both left and right encoders
// ---------------------------------------------------
void readShaftPositions(float *angleLeft, float *angleRight)
{
    float tmpAngle;

    // LEFT
    if (singleEncReading(ENCODER_LEFT_ADDR, &tmpAngle) != HAL_OK) {
        printf("Warning(I2C): Could not read left encoder\r\n");
        *angleLeft = 0.0f;   // SCUTTLE's way to address this error
    } else {
        // invert the left reading: angle0 = 360.0 - rawAngle
        float inv = 360.0f - tmpAngle;
        // round to 0.1
        float tmp = inv * 10.0f;
        int tmpInt = (int)(tmp + 0.5f); // works for non-negative values (angles are 0..360)
        *angleLeft = (float)tmpInt / 10.0f;
    }


    // RIGHT
    if (singleEncReading(ENCODER_RIGHT_ADDR, &tmpAngle) != HAL_OK) {
        printf("Warning(I2C): Could not read right encoder\r\n");
        *angleRight = 0.0f;   // SCUTTLE's way to address this error
    } else {
        *angleRight = tmpAngle;
    }
}


void I2C_Scan(void)
{
    printf("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
//    	uint8_t reg = 0xFE;
//    	uint8_t data;
//    	HAL_I2C_Master_Transmit(&hi2c1, 0x41 << 1, &reg, 1, 100);
//    	HAL_I2C_Master_Receive(&hi2c1, 0x41 << 1, &data, 1, 100);

        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
            printf("Found device at 0x%02X\r\n", addr);
    }
}
