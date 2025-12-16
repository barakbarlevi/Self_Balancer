#include "motors.h"

extern TIM_HandleTypeDef htim3;

/**
 * @brief Initialize a motor with a specific configuration and side.
 *        Sets up the minimum forward and backward PWM values based on the robot
 *        configuration and whether the motor is on the left or right side.
 *
 * @param Motor: Pointer to the Motor_t structure to initialize.
 * @param configuration: Robot configuration type from the Robot_Configuration enum.
 * @param is_left_motor: Set to 1 if the motor is the left motor, 0 if it is the right motor.
 */
void Motor_Init(Motor_t *Motor, Robot_Configuration configuration, uint8_t is_left_motor) {

	Motor->robotConfiguration = configuration;
	Motor->is_left_motor = is_left_motor;

	if(is_left_motor == 1) {

		switch(configuration) {

			case WHEELS_IN_AIR:
				Motor->Min_PWM_Forward = Min_PWM_Left_Forward_IN_AIR;
				Motor->Min_PWM_Backward = Min_PWM_Left_Backward_IN_AIR;
				break;

			case PID_Tuning_Safe:
				Motor->Min_PWM_Forward = Min_PWM_Left_Forward_Tuning_Safe;
				Motor->Min_PWM_Backward = Min_PWM_Left_Backward_Tuning_Safe;
				break;

			case Only_Ballast:
				Motor->Min_PWM_Forward = Min_PWM_Left_Forward_Only_Ballast;
				Motor->Min_PWM_Backward = Min_PWM_Left_Backward_Only_Ballast;
				break;

			case Full_Configuration:
				Motor->Min_PWM_Forward = Min_PWM_Left_Forward_Full;
				Motor->Min_PWM_Backward = Min_PWM_Left_Backward_Full;
				break;
		}

	} else if(is_left_motor == 0) {

		switch(configuration) {

			case WHEELS_IN_AIR:
				Motor->Min_PWM_Forward = Min_PWM_Right_Forward_IN_AIR;
				Motor->Min_PWM_Backward = Min_PWM_Right_Backward_IN_AIR;
				break;

			case PID_Tuning_Safe:
				Motor->Min_PWM_Forward = Min_PWM_Right_Forward_Tuning_Safe;
				Motor->Min_PWM_Backward = Min_PWM_Right_Backward_Tuning_Safe;
				break;

			case Only_Ballast:
				Motor->Min_PWM_Forward = Min_PWM_Right_Forward_Only_Ballast;
				Motor->Min_PWM_Backward = Min_PWM_Right_Backward_Only_Ballast;
				break;

			case Full_Configuration:
				Motor->Min_PWM_Forward = Min_PWM_Right_Forward_Full;
				Motor->Min_PWM_Backward = Min_PWM_Right_Backward_Full;
				break;
		}
	}

	Motor->pwm = 0;
	Motor->last_pwm = 0;
}

/**
 * @brief Set the PWM output for a motor.
 *
 * This function updates the timer compare registers to drive the motor
 * forward or backward depending on the sign of the integer speed parameter.
 */
void setPWM(Motor_t *Motor, int speed) {

	if(Motor->is_left_motor == 1) {
		if (speed >= 0) {
			//MotorB_Forward(speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); // IN3 = PWM
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);     // IN4 = 0
		}
		else {
			//MotorB_Backward(-speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speed); // IN4 = PWM
		}
	}

	else if(Motor->is_left_motor == 0) {
		if (speed >= 0) {
			//MotorA_Forward(speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); // IN1 = PWM
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);     // IN2 = 0
		} else {
			//MotorA_Backward(-speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed); // IN2 = PWM
		}
	}
}

/**
 * @brief Stop both motors by setting the compare registers to 0 on all TIM3 channels.
 */
void Motors_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}


/**
 * @brief Adjust a PWM value to account for the motor's minimum effective threshold.
 *
 * MIN_PWM_* is the minimal pwm command that will overcome DC motor dead band and the friction in the wheel assemblies
 * to generate a continuous, steady rotation of the wheels.
 *
 * @param Motor Pointer to the Motor_t structure representing the motor.
 * @param pwm Input PWM value (can be positive, negative, or zero).
 *
 * @retval Adjusted PWM value that includes the deadzone compensation.
 */
int apply_deadzone(Motor_t *Motor, int pwm)
{
    if (pwm == 0) {
        return 0;
    }

    if (pwm > 0) { return (pwm + Motor->Min_PWM_Forward); }
    else {return (pwm + Motor->Min_PWM_Backward); }
}

/*
void setPWM_MotorRight(int speed) {
	if (speed >= 0) {
		//MotorA_Forward(speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); // IN1 = PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);     // IN2 = 0
	}
	else {
		//MotorA_Backward(-speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed); // IN2 = PWM
	}
}

void setPWM_MotorLeft(int speed) {
	if (speed >= 0) {
		//MotorB_Forward(speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); // IN3 = PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);     // IN4 = 0
	}
	else {
		//MotorB_Backward(-speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speed); // IN4 = PWM
	}
}
*/

