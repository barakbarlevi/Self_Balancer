#include "imu.h"

static uint8_t rx[6*6] = {0};	// Receive buffer for IMU data

/* 48-bit mode operations, see 6.6.2 in the datasheet */
static uint8_t cmd_Read_ACC_X1[6] = {0x01, 0x08, 0x00, 0x00, 0x00, 0xF6};
static uint8_t cmd_Read_ACC_Y1[6] = {0x01, 0x48, 0x00, 0x00, 0x00, 0x01};
static uint8_t cmd_Read_ACC_Z1[6] = {0x01, 0x88, 0x00, 0x00, 0x00, 0x37};
static uint8_t cmd_Read_RATE_X1[6] = {0x00, 0x48, 0x00, 0x00, 0x00, 0xAC};
static uint8_t cmd_Read_RATE_Y1[6] = {0x00, 0x88, 0x00, 0x00, 0x00, 0x9A};
static uint8_t cmd_Read_RATE_Z1[6] = {0x00, 0xC8, 0x00, 0x00, 0x00, 0x6D};
/* -------------------------------------------------- */

/* last_* are used in function single_IMU_reading() to filter noisy data */
static float last_ax = 0.0f;
static float last_ay = 0.0f;
static float last_az = 0.0f;
static float last_gx = 0.0f;
static float last_gy = 0.0f;
static float last_gz = 0.0f;
/* --------------------------------------------------------------------- */

// SPI 48-bit transfer (3x16-bit)
/**
 * @brief  NOTE: See datasheet section 6.1 for the illustrated request/response way of operation.
 * Each transfer has two phases -
 * First phase - request and response to the previous command.
 * Second phase - next request and response to the request of the first phase.
 * The first response after reset is undefined and should be discarded.
 *
 * @param  hspi: Pointer to the SPI handle used for communication.
 * @param  tx: Pointer to a 6-byte transmit buffer containing data to send.
 * @param  rx: Pointer to a 6-byte receive buffer where received data is stored.
 */
static HAL_StatusTypeDef IMU_SPI_Transfer48(SPI_HandleTypeDef *hspi, uint8_t tx[6], uint8_t rx[6])
{
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t word;

    /* Transmit and receive operations are between a RESET and SET of Chip Select IMU line */
    IMU_Select();
    for (int i = 0; i < 3 && status == HAL_OK; i++)
    {
        word = (tx[2*i] << 8) | tx[2*i + 1];
        status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)&word, (uint8_t*)&word, 1, HAL_MAX_DELAY);
        rx[2*i]     = word >> 8;
        rx[2*i + 1] = word & 0xFF;
    }
    IMU_Deselect();
    /* ----------------------------------------------------------------------------------- */

    return status;
}

/**
 * @brief  Build 48-bit read command frame for given TA (0x00–0x3FF)
 * NOTE:   See frame format in section 6.2 in the datasheet
 * @param  TA: Target Address. For example, status register on the IMU. Any register chosen by the user
 * @param  cmd: Pointer to a 6-byte buffer where the generated command is stored
 */
static void IMU_BuildReadCommand(uint16_t TA, uint8_t cmd[6])
{
    uint64_t frame = 0;

    // TA bits [47:38]
    frame |= ((uint64_t)(TA & 0x3FF) << 38);

    // RW = 0 (read)
    // bit36 = 0
    // FT = 1 (bit35)
    frame |= ((uint64_t)1 << 35);

    // Reserved + DATAI = 0

    uint64_t data = frame & 0xFFFFFFFFFF00ULL; // ignore CRC bits
	uint8_t crc = 0xFF;
	for (int i = 47; i >= 0; i--)
	{
		uint8_t data_bit = (data >> i) & 0x01;
		if (crc & 0x80)
			crc = ((crc << 1) ^ 0x2F) ^ data_bit;
		else
			crc = (crc << 1) | data_bit;
	}

    frame |= crc;  // Bits 7:0 = CRC8

    // Split into 6 bytes, MSB first
    for (int i = 0; i < 6; i++)
        cmd[i] = (uint8_t)((frame >> ((5 - i) * 8)) & 0xFF);
}


/**
 * @brief Extract INFO[19:0] (bits 27..8) from 48-bit MISO frame
 * @param cmd: Pointer to a 6-byte receive buffer
 */
static uint32_t IMU_ExtractInfo20(const uint8_t rx[6])
{
    uint64_t frame = 0;
    for (int i = 0; i < 6; i++)
        frame = (frame << 8) | rx[i];

    return (frame >> 8) & 0xFFFFF; // bits 27..8
}

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
 * @brief  Asserts the IMU chip-select (NSS) line. Pulls the NSS pin low to select the IMU for SPI communication.
 * Must be called before starting an SPI transaction with the IMU.
 */
void IMU_Select()   { HAL_GPIO_WritePin(IMU_NSS_GPIO_Port, IMU_NSS_Pin, GPIO_PIN_RESET); }


/**
 * @brief  Deasserts the IMU chip-select (NSS) line. Pulls the NSS pin high to deselect the IMU and end the SPI transaction.
 * Must be called after completing an SPI transaction with the IMU.
 */
void IMU_Deselect() { HAL_GPIO_WritePin(IMU_NSS_GPIO_Port, IMU_NSS_Pin, GPIO_PIN_SET); }


/**
 * @brief  Implement startup sequence as described in section 5.2 in IMU's datasheet
 * @param  hspi: Pointer to the SPI handle used for IMU communication.
 */
void IMU_StartUpSequence(SPI_HandleTypeDef *hspi) {
    uint8_t rx[6];

    // 1. Power on (assume external supply active)

    // 2. Wait 1 ms for voltage stabilization
    HAL_Delay(3);

    // 3. Wait 32 ms for NVM read / SPI start up
    HAL_Delay(50);

    // 4. Defaults used, skip user control configuration

    // 5. Write EN_SENSOR = 1 (0x0D68000001D3)
    uint8_t en_sensor[6] = {0x0D, 0x68, 0x00, 0x00, 0x01, 0xD3};
    IMU_SPI_Transfer48(hspi, en_sensor, rx); // First response is meaningless

    // 6. Wait 215 ms
    HAL_Delay(260);

    // 7. Read all status registers once
    uint16_t status_regs[STATUS_REG_COUNT];
	uint8_t status_reg_addrs[STATUS_REG_COUNT] = {
		0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
		0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
	};

	for (int i = 0; i < STATUS_REG_COUNT; i++) {
		uint8_t ok = 0;
		int retries = 0;

		while (!ok && retries < MAX_STATUS_RETRIES) {
			uint8_t cmd[6];
			IMU_BuildReadCommand(status_reg_addrs[i], cmd); // build 48-bit read command

			IMU_SPI_Transfer48(hspi, cmd, rx); // send command & receive full 48-bit response

			// extract 16-bit register value from received 48-bit frame
			// INFO[15:0] assumed stored in bits 8..23 (depends on IMU frame layout)
			status_regs[i] = ((uint16_t)rx[1] << 8) | rx[2];

			// simple OK check: all bits 1 in the 16-bit register
			if (status_regs[i] == 0xFFFF) {
				ok = 1; // status is OK
			} else {
				retries++;
				HAL_Delay(5); // short delay before retry
			}
		}
	}


    // 8. Write EOI = 1 (0x0D680000038D)
    uint8_t eoi[6] = {0x0D, 0x68, 0x00, 0x00, 0x03, 0x8D};
    IMU_SPI_Transfer48(hspi, eoi, rx);

    // 9. Wait 3 ms
    HAL_Delay(10);

    // 10. Read all status registers twice and verify OK
    for (int read_twice = 0; read_twice < 2; read_twice++) {
        for (int i = 0; i < STATUS_REG_COUNT; i++) {
            uint8_t ok = 0;
            int retries = 0;

            while (!ok && retries < MAX_STATUS_RETRIES) {
                uint8_t cmd[6];
                IMU_BuildReadCommand(status_reg_addrs[i], cmd); // build 48-bit read command

                IMU_SPI_Transfer48(hspi, cmd, rx); // send command & receive full 48-bit response

                // extract 16-bit register value from received 48-bit frame
                // INFO[15:0] assumed stored in bits 8..23 (depends on IMU frame layout)
                status_regs[i] = ((uint16_t)rx[1] << 8) | rx[2];

                // simple OK check: all bits 1 in the 16-bit register
                if (status_regs[i] == 0xFFFF) {
                    ok = 1; // status is OK
                } else {
                    retries++;
                    HAL_Delay(5); // short delay before retry
                }
            }
        }
     }


    HAL_Delay(10);

    /* 11. Read all configuration registers - Not needed
    for (i = 0; i < 6; i++) {
        IMU_SPI_Transfer48(CONFIG_REG_CMDS[i], rx);
    }
    */

}


/**
 * @brief  Reads and prints the IMU 60-bit serial number over SPI.
           The serial number is composed of three consecutive 20-bit register values stored
           at (0x3D, 0x3E, 0x3F).
 */
void IMU_ReadSerialNumber(SPI_HandleTypeDef *hspi)
{
    const uint16_t sn_addr[3] = {0x3D, 0x3E, 0x3F};
    uint8_t tx[6], rx[6];
    uint32_t sn_part[3] = {0};

    for (int i = 0; i < 3 + 1; i++)
    {
    	if ( i == 3) {
    		IMU_BuildReadCommand(sn_addr[2], tx);
    	} else {
        IMU_BuildReadCommand(sn_addr[i], tx);
    	}

        if (IMU_SPI_Transfer48(hspi, tx, rx) != HAL_OK)
        {
            printf("SPI transfer failed for SN_ID%d\r\n", i+1);
            return;
        }

        sn_part[i] = IMU_ExtractInfo20(rx);
    }

    // Combine 3 × 20-bit values into a 60-bit serial number
    uint64_t serial = ((uint64_t)sn_part[0] << 40) |
                      ((uint64_t)sn_part[1] << 20) |
                       (uint64_t)sn_part[2];

    printf("IMU Serial Number: 0x%015llX\r\n", (unsigned long long)serial);
}


/**
  * @brief  Sign-extends a 20-bit signed value to a 32-bit signed integer.
  *         Converts a 20-bit two’s-complement number into a properly
  *         sign-extended 32-bit value.
  *
  * @param  raw: Unsigned 20-bit value to be sign-extended (bits [19:0] valid).
  * @retval Signed 32-bit integer with correct sign extension applied.
  */
int32_t sign_extend_20bit(uint32_t raw)
{
    if (raw & (1 << 19))
        raw |= 0xFFF00000; // sign-extend bit 19
    return (int32_t)raw;
}


/**
  * NOTE: This is an old, unused function. All telemetry is send in an updated function send_telem_frame_over_STLink().
  * @brief  Packs IMU sensor data into a UART frame and transmits it.
  *
  * @param  huart: Pointer to the UART handle used for transmission.
  * @param  imu:   Pointer to the IMU_Data_t structure containing sensor readings.
  */
void send_IMU_frame(UART_HandleTypeDef *huart, IMU_Data_t *imu)
{
    uint8_t frame[2 + 6 * 4]; // 2-byte header + 6 floats (4 bytes each)
    uint16_t header = IMU_DATA_FRAME_HEADER;

    // write header (little endian)
    frame[0] = (uint8_t)(header & 0xFF);
    frame[1] = (uint8_t)(header >> 8);

    pack_float_le(&frame[2],  imu->ax);
	pack_float_le(&frame[6],  imu->ay);
	pack_float_le(&frame[10], imu->az);
	pack_float_le(&frame[14], imu->gx);
	pack_float_le(&frame[18], imu->gy);
	pack_float_le(&frame[22], imu->gz);

    HAL_UART_Transmit(huart, frame, sizeof(frame), HAL_MAX_DELAY);
}



/**
  * @brief  Performs a single reading of the IMU's accelerometer and gyroscope data.
  *         Reads raw 20-bit sensor values via SPI, applies sign extension,
  *         converts to physical units (m/s² and deg/s), and enforces sensor limits.
  *
  * @param  hspi: Pointer to the SPI handle used to communicate with the IMU.
  * @param  imu:  Pointer to an IMU_Data_t structure where the results will be stored.
  */
void single_IMU_reading(SPI_HandleTypeDef *hspi, IMU_Data_t *imu) {

	static int first_read=1;

	/**
	 * NOTE: See datasheet section 6.1 for the illustrated request/response way of operation.
	 * Each transfer has two phases -
	 * First phase - request and response to the previous command.
	 * Second phase - next request and response to the request of the first phase.
	 * The first response after reset is undefined and should be discarded.
	 */
	IMU_SPI_Transfer48(hspi, cmd_Read_ACC_X1, rx + 30);

	if (first_read) {
		for (int i = 0; i < 6; i++) {
		   rx[30 + i] = 0x00;
		}
		first_read = 0;
	}

	  IMU_SPI_Transfer48(hspi, cmd_Read_ACC_Y1, rx);
	  IMU_SPI_Transfer48(hspi, cmd_Read_ACC_Z1, rx+6);
	  IMU_SPI_Transfer48(hspi, cmd_Read_RATE_X1, rx+12);
	  IMU_SPI_Transfer48(hspi, cmd_Read_RATE_Y1, rx+18);
	  IMU_SPI_Transfer48(hspi, cmd_Read_RATE_Z1, rx+24);

	  int32_t ax_raw = sign_extend_20bit(IMU_ExtractInfo20(rx));
	  int32_t ay_raw = sign_extend_20bit(IMU_ExtractInfo20(rx+6));
	  int32_t az_raw = sign_extend_20bit(IMU_ExtractInfo20(rx+12));
	  int32_t gx_raw = sign_extend_20bit(IMU_ExtractInfo20(rx+18));
	  int32_t gy_raw = sign_extend_20bit(IMU_ExtractInfo20(rx+24));
	  int32_t gz_raw = sign_extend_20bit(IMU_ExtractInfo20(rx+30));

	  /*--- Scale raw values to get the physical data ---*/
	  float ax = ((float)ax_raw) / ACC_SENSITIVITY;
	  float ay = ((float)ay_raw) / ACC_SENSITIVITY;
	  float az = ((float)az_raw) / ACC_SENSITIVITY;
	  float gx = ((float)gx_raw) / GYRO_SENSITIVITY;
	  float gy = ((float)gy_raw) / GYRO_SENSITIVITY;
	  float gz = ((float)gz_raw) / GYRO_SENSITIVITY;
	  /*-------------------------------------------------*/

	  /* --- store the last valid reading in case new values exceed safe limits ---*/
	  last_ax = (ax >= -ACC_MAX && ax <= ACC_MAX) ? ax : last_ax;
	  last_ay = (ay >= -ACC_MAX && ay <= ACC_MAX) ? ay : last_ay;
	  last_az = (az >= -ACC_MAX && az <= ACC_MAX) ? az : last_az;

	  last_gx = (gx >= -GYRO_MAX && gx <= GYRO_MAX) ? gx : last_gx;
	  last_gy = (gy >= -GYRO_MAX && gy <= GYRO_MAX) ? gy : last_gy;
	  last_gz = (gz >= -GYRO_MAX && gz <= GYRO_MAX) ? gz : last_gz;
	  /* --------------------------------------------------------------------------*/

	  /*--- Store values in the IMU data structure ---*/
	  imu->ax = last_ax; imu->ay = last_ay; imu->az = last_az;
	  imu->gx = last_gx; imu->gy = last_gy; imu->gz = last_gz;
	  /*----------------------------------------------*/

	  //printf("AX=%.3f AY=%.3f AZ=%.3f GX=%.3f GY=%.3f GZ=%.3f\r\n",last_ax, last_ay, last_az, last_gx, last_gy, last_gz); // Optional

}
