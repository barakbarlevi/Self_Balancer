/**
 * Note: At first, a two state kalman filter for an additional bias state was implemented. This filter provided poor results,
 * perhaps because the SCH16T IMU already comes with biases calibrated. This lead to a single-state kalman filter implementation,
 * that can be used for pitch/yaw estimations. The 1x1 filters give results which are almost exactly similar to simpler complementary
 * filters with a parameter alpha. The code for higher state filters was commented out and can be re-used for future improvements.
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "stm32f3xx_hal.h"

#define PITCH_KAL1x1_FRAME_HEADER_1 0x66
#define PITCH_KAL1x1_FRAME_HEADER_2 0xAB
#define YAW_KAL1x1_FRAME_HEADER_1 0x77
#define YAW_KAL1x1_FRAME_HEADER_2 0xCD
#define KALMAN1x1_FRAME_PAYLOAD_SIZE 20   // 5 float of 4 bytes each
#define KALMAN1x1_FRAME_TOTAL_SIZE (2 + KALMAN1x1_FRAME_PAYLOAD_SIZE)

/*
#define PITCH_KALMAN_FRAME_HEADER_1 0x33
#define PITCH_KALMAN_FRAME_HEADER_2 0xCC
#define YAW_KALMAN_FRAME_HEADER_1 0x22
#define YAW_KALMAN_FRAME_HEADER_2 0xDD
#define KALMAN_FRAME_PAYLOAD_SIZE 32   // 8 floats of 4 bytes each
#define KALMAN_FRAME_TOTAL_SIZE (2 + KALMAN_FRAME_PAYLOAD_SIZE)
*/

typedef enum {
    KALMAN_PITCH,
    KALMAN_YAW
} KalmanType_t;


typedef struct {
	KalmanType_t type;   // pitch or yaw

    float Q_state;   // Process noise variance
    float R_measure; // Measurement noise variance

    float state_estimate;     // The angle estimated by the 1x1 Kalman filter
    float P;         		  // Error covariance matrix - 1x1

    float K0;        // Kalman gain
	float measurement_residual;
} Kalman1x1_t;

/*
typedef struct {
	KalmanType_t type;

    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float P[2][2];

    float K0, K1;
	float measurement_residual;
} Kalman_t;
*/

void Kalman1x1_Init(Kalman1x1_t* kalman, KalmanType_t type, float Qstate, float Rmeasure);
void Kalman1x1_Update(Kalman1x1_t* kalman, float newAngle, float newRate, float dt);
void send_Kalman1x1_frame(UART_HandleTypeDef *huart, Kalman1x1_t* kalman, float dt);

/*
void Kalman_Init(Kalman_t* kalman, KalmanType_t type, float Qangle, float Qbias, float Rmeasure);
void Kalman_Update(Kalman_t* kalman, float newAngle, float newRate, float dt);
void send_Kalman_frame(UART_HandleTypeDef *huart, Kalman_t* kalman, float dt);
*/

#endif /* INC_KALMAN_H_ */
