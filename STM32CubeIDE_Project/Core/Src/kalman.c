#include "kalman.h"


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
 * @brief Initializes a 1x1 Kalman filter for pitch or yaw estimation.
 *
 * This function sets the filter type, process noise, measurement noise, and
 * initializes the state estimate, error covariance, Kalman gain, and residuals to default values.
 *
 * @param kalman:   Pointer to the Kalman1x1_t structure to initialize.
 * @param type:     Type of the filter (KALMAN_PITCH or KALMAN_YAW).
 * @param Qstate:   Process noise variance for the state (affects prediction confidence).
 * @param Rmeasure: Measurement noise variance (affects update weighting).
 */
void Kalman1x1_Init(Kalman1x1_t* kalman, KalmanType_t type, float Qstate, float Rmeasure) {
	kalman->type = type;
	kalman->Q_state = Qstate;
	kalman->R_measure = Rmeasure;
    kalman->state_estimate = 0.0f;
    kalman->P = 1.0f;
	kalman->K0 = 0.0f;
	kalman->measurement_residual  = 0.0f;
}


/**
 * @brief Performs a single update step of a 1x1 Kalman filter for pitch or yaw.
 *
 * @param kalman: Pointer   to the Kalman1x1_t structure representing the filter state.
 * @param newMeasurement_Z: The new measurement (angle for pitch, rate for yaw).
 * @param newInput_U:       The control input or rate used for prediction (gyro value).
 * @param dt:               Time step in seconds since the last update.
 */
void Kalman1x1_Update(Kalman1x1_t* kalman, float newMeasurement_Z, float newInput_U, float dt) {

    // Predict Step. Based on the input (gyro rate) and the filter type
    if(kalman->type == KALMAN_PITCH) {
        kalman->state_estimate += dt * newInput_U;

    } else if (kalman->type == KALMAN_YAW) {
        kalman->state_estimate = newInput_U;
    }

    // Propagate the filter’s confidence forward in time
    kalman->P += kalman->Q_state * dt;

    // Compute Innovation (the difference between the actual measurement and the state_estimate)
    float y = newMeasurement_Z - kalman->state_estimate;

    // Compute Kalman gain
    float S = kalman->P + kalman->R_measure;
    float K_gain = kalman->P / S;

    // State Update (Correction): Apply correction to the state estimate
    kalman->state_estimate += K_gain * y;

    // Covariance Update
    kalman->P *= (1.0 - K_gain);

    // Store results
    kalman->K0 = K_gain;
    kalman->measurement_residual  = y;
}

/**
  * NOTE: This is an old, UNUSED function. All telemetry is send in an updated function send_telem_frame_over_STLink().
  * @brief  Packs Kalman1x1 sensor data into a UART frame and transmits it.
  *
  * @param  huart: Pointer to the UART handle used for transmission.
  * @param  kalman: Pointer to the Kalman1x1_t structure containing state estimations.
  * @param  dt: time difference from the last frame sending
  */
void send_Kalman1x1_frame(UART_HandleTypeDef *huart, Kalman1x1_t* kalman, float dt)
{
    uint8_t kalman1x1_frame[KALMAN1x1_FRAME_TOTAL_SIZE]; // 2 + 5 floats = 2 + 5*4

    // write header (little endian)
    if(kalman->type == KALMAN_PITCH) {
    	kalman1x1_frame[0] = (uint8_t)PITCH_KAL1x1_FRAME_HEADER_1;
		kalman1x1_frame[1] = (uint8_t)PITCH_KAL1x1_FRAME_HEADER_2;
    } else if (kalman->type == KALMAN_YAW) {
    	kalman1x1_frame[0] = (uint8_t)YAW_KAL1x1_FRAME_HEADER_1;
		kalman1x1_frame[1] = (uint8_t)YAW_KAL1x1_FRAME_HEADER_2;
    }

    pack_float_le(&kalman1x1_frame[2],  dt);
	pack_float_le(&kalman1x1_frame[6],  kalman->state_estimate);
	pack_float_le(&kalman1x1_frame[10], kalman->K0);
	pack_float_le(&kalman1x1_frame[14], kalman->P);
	pack_float_le(&kalman1x1_frame[18], kalman->measurement_residual);

    HAL_UART_Transmit(huart, kalman1x1_frame, sizeof(kalman1x1_frame), HAL_MAX_DELAY);
}


/**
 * Note: At first, a two state kalman filter for an additional bias state was implemented. This filter provided poor results,
 * perhaps because the SCH16T IMU already comes with biases calibrated. This lead to a single-state kalman filter implementation,
 * that can be used for pitch/yaw estimations. The 1x1 filters give results which are almost exactly similar to simpler complementary
 * filters with a parameter alpha. The code for higher state filters was commented out and can be re-used for future improvements.
 */
/*
void Kalman_Init(Kalman_t* kalman, KalmanType_t type, float Qangle, float Qbias, float Rmeasure) {
	kalman->type = type;

    //kalman->Q_angle = 0.001f;
    //kalman->Q_bias = 0.003f;
    //kalman->R_measure = 0.03f;
	kalman->Q_angle = Qangle;
	kalman->Q_bias = Qbias;
	kalman->R_measure = Rmeasure;

    kalman->angle = 0.0f;
    kalman->bias = 0.0f;

    kalman->P[0][0] = 1.0f;  // large uncertainty in angle
	kalman->P[0][1] = 0.0f;
	kalman->P[1][0] = 0.0f;
	kalman->P[1][1] = 1.0f;  // large uncertainty in bias

	// Optional: clear debug variables
	kalman->K0 = 0.0f;
	kalman->K1 = 0.0f;
	kalman->measurement_residual  = 0.0f;
}

void Kalman_Update(Kalman_t* kalman, float newAngle, float newRate, float dt) {
	float y;

	// Predict
	float rate = newRate - kalman->bias;
	kalman->angle += dt * rate;
	// Update error covariance matrix
	kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += kalman->Q_bias * dt;

	// Innovation / residual
   	y = newAngle - kalman->angle;

   	// Compute Kalman gains
   	float S = kalman->P[0][0] + kalman->R_measure;
   	float K0 = kalman->P[0][0] / S;
   	float K1 = kalman->P[1][0] / S;

   	// Update estimate
   	kalman->angle += K0 * y;
   	kalman->bias  += K1 * y;

   	// Update error covariance matrix
   	float P00_temp = kalman->P[0][0];
   	float P01_temp = kalman->P[0][1];
   	kalman->P[0][0] -= K0 * P00_temp;
   	kalman->P[0][1] -= K0 * P01_temp;
   	kalman->P[1][0] -= K1 * P00_temp;
   	kalman->P[1][1] -= K1 * P01_temp;
   	kalman->K0 = K0;
	kalman->K1 = K1;
	kalman->measurement_residual  = y;
}


void send_Kalman_frame(UART_HandleTypeDef *huart, Kalman_t* kalman, float dt)
{
    uint8_t kalman_frame[KALMAN_FRAME_TOTAL_SIZE]; // 2 + 8 floats = 2 + 8*4

    // write header (little endian)
    if(kalman->type == KALMAN_PITCH) {
    	kalman_frame[0] = (uint8_t)PITCH_KALMAN_FRAME_HEADER_1;
		kalman_frame[1] = (uint8_t)PITCH_KALMAN_FRAME_HEADER_2;
    } else if (kalman->type == KALMAN_YAW) {
    	kalman_frame[0] = (uint8_t)YAW_KALMAN_FRAME_HEADER_1;
		kalman_frame[1] = (uint8_t)YAW_KALMAN_FRAME_HEADER_2;
    }


    pack_float_le(&kalman_frame[2],  dt);
	pack_float_le(&kalman_frame[6],  kalman->angle);
	pack_float_le(&kalman_frame[10], kalman->bias);
	pack_float_le(&kalman_frame[14], kalman->K0);
	pack_float_le(&kalman_frame[18], kalman->K1);
	pack_float_le(&kalman_frame[22], kalman->P[0][0]);
	pack_float_le(&kalman_frame[26], kalman->P[1][1]);
	pack_float_le(&kalman_frame[30], kalman->measurement_residual);

    HAL_UART_Transmit(huart, kalman_frame, sizeof(kalman_frame), HAL_MAX_DELAY);
}
*/


