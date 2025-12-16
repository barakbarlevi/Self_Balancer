/*
 * Motor control interface for a two-wheeled robot using STM32 timers.
 *
 * This module provides functions and structures to initialize and control the left and right DC
 * motors of the robot. It handles multiple robot physical configurations, assuming it might change mass and dimensions.
  */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f3xx_hal.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;

#define MAX_PWM_ALLOWED htim3.Instance->ARR
#define MAX_EFFECTIVE_PWM (1.0f * htim3.Instance->ARR)
#define MAX_PWM_STEP 1.0f * htim3.Instance->ARR

/**
 * MIN_PWM_* is the minimal pwm command that will overcome DC motor dead band and the friction in the wheel assemblies
 * to generate a continuous, steady rotation of the wheels. It depends on the configuration under test. For example,
 * it would be easier to rotate the wheels when the robot's wheels aren't touching the floor, or when no additional
 * mass is added on top of it.
 * They depend heavily on the motor PWM frequency, and are in the range [-htim3.Instance->ARR, 0] for backward rotation and
 * [0, htim3.Instance->ARR] for forward rotation.
 */
#define Min_PWM_Left_Forward_IN_AIR 200
#define Min_PWM_Right_Forward_IN_AIR 190
#define Min_PWM_Left_Backward_IN_AIR -190
#define Min_PWM_Right_Backward_IN_AIR -190

#define Min_PWM_Left_Forward_Tuning_Safe 250
#define Min_PWM_Right_Forward_Tuning_Safe 250
#define Min_PWM_Left_Backward_Tuning_Safe -210
#define Min_PWM_Right_Backward_Tuning_Safe -210

#define Min_PWM_Left_Forward_Only_Ballast 200
#define Min_PWM_Left_Backward_Only_Ballast -190
#define Min_PWM_Right_Forward_Only_Ballast 190
#define Min_PWM_Right_Backward_Only_Ballast -190

#define Min_PWM_Left_Forward_Full 1
#define Min_PWM_Right_Forward_Full 1
#define Min_PWM_Left_Backward_Full -1
#define Min_PWM_Right_Backward_Full -1

typedef enum {
	WHEELS_IN_AIR,
	PID_Tuning_Safe,
	Only_Ballast,
	Full_Configuration
} Robot_Configuration;

typedef struct {

	Robot_Configuration robotConfiguration;
	int Min_PWM_Forward;
	int Min_PWM_Backward;
	int pwm;
	int last_pwm;
	uint8_t is_left_motor;

} Motor_t;

void Motors_Stop(void);

void Motor_Init(Motor_t *Motor, Robot_Configuration Robot_Configuration, uint8_t is_left_motor);
void setPWM(Motor_t *Motor, int speed);
int apply_deadzone(Motor_t *Motor, int pwm);

void do_MotorTrials(Motor_t* leftMotor, Motor_t* rightMotor); // Implemented in main.c
void set_PWM_for_balancing(Motor_t* leftMotor, Motor_t* rightMotor, float u_forward, float u_turn); // Implemented in main.c

/*
void MotorA_Forward(uint16_t speed);
void MotorA_Backward(uint16_t speed);
void MotorB_Forward(uint16_t speed);
void MotorB_Backward(uint16_t speed);
void setPWM_MotorLeft(int speed);
void setPWM_MotorRight(int speed);
int apply_deadzone(int pwm, Robot_Configuration configuration, int side);  // 0 - left , 1 - right
*/


#endif /* INC_MOTORS_H_ */
