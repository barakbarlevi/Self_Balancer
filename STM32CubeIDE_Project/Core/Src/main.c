/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "imu.h"
#include "motors.h"
#include "chassis.h"
#include "kalman.h"
#include "PID.h"
#include "stm_pi_uart1_link.h"
#include "stm_uart2_link.h"
#include "operator_commands.h"
#include "control_quantities.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

stm_pi_uart1_link_t stm_pi_uart1_link;
ChassisData_t chassisData;
IMU_Data_t imu;
Kalman1x1_t pitch_kalman1x1, yaw_kalman1x1;
PID_t pitch_angle_pid, speed_pid, yaw_angle_pid;
Motor_t leftMotor, rightMotor;
ControlQuantities_t controlQuantities;
operator_commands_t operator_commands;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* ------- Initializations -------*/
  STM_Pi_UART1_Link_Init(&stm_pi_uart1_link);
  ChassisData_Init(&chassisData);

  Kalman1x1_Init(&pitch_kalman1x1, KALMAN_PITCH, 0.012, 0.03f);
  Kalman1x1_Init(&yaw_kalman1x1, KALMAN_YAW, 0.012, 0.001f);

  PID_Init(&pitch_angle_pid, PID_TYPE_PITCH_ANGLE, 0.0f, 0.0f, 0.0f); // Tested OK for preliminary robot configuration - (220.0f, 0.0f, 45.0f) , (35.0f, 0.0f, 5.7f)
  PID_Init(&speed_pid, PID_TYPE_SPEED, 0.0f, 0.0f, 0.0f); // Tested OK for preliminary robot configuration - (3.5f, 1.5f, 0.0f)
  PID_Init(&yaw_angle_pid, PID_TYPE_YAW_ANGLE, 0.0f, 0.0f, 0.0f);

  Motor_Init(&leftMotor, Full_Configuration, 1);
  Motor_Init(&rightMotor, Full_Configuration, 0);

  Control_Quantites_Init(&controlQuantities);
  Operator_Commands_Init(&operator_commands);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  IMU_Deselect();
  IMU_StartUpSequence(&hspi1);
  //IMU_ReadSerialNumber();

  /* -------------------------------*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_UART_Receive_IT(&huart1, stm_pi_uart1_link.rx_buffer, PI_TO_STM_FRAME_TOTAL_SIZE);

  HAL_Delay(2000);

  HAL_UART_Transmit_IT(&huart1, &stm_pi_uart1_link.ready_byte, 1);

  uint32_t last_control_tick = HAL_GetTick();

  while (1)
  {
	  uint32_t now = HAL_GetTick();
	  if (now - last_control_tick >= controlQuantities.desired_control_period_ms) {

		  /* --- Compute actual control loop frequency (dt_actual) ---*/
		  uint32_t current_tick = now;
		  controlQuantities.dt_actual = (current_tick - last_control_tick) * 0.001f;
		  last_control_tick = current_tick;
		  /* ---------------------------------------------------------*/

		  check_for_updates_from_tuning_setup();

		  float u_forward = 0.0f;
		  float u_turn = 0.0f;

		  compute_controllers_commands(&u_forward, &u_turn);
		  set_PWM_for_balancing(&leftMotor, &rightMotor, u_forward, u_turn);
		  //do_MotorTrials(&leftMotor, &rightMotor);

		  send_telem_frame_over_STLink(&huart2,
		  			  	  	  	  	    pitch_kalman1x1.state_estimate,
										imu.gx,
										controlQuantities.u_pitch_angle,
		  								chassisData.xdot,
		  								chassisData.left_wheel_speed_rad_sec,
		  								chassisData.right_wheel_speed_rad_sec,
		  								controlQuantities.u_speed_hold,
										leftMotor.pwm,
										rightMotor.pwm,
		  								u_forward,
										chassisData.psidot,
		  								chassisData.yaw_angle,
		  								yaw_kalman1x1.state_estimate,
		  								imu.gz,
										0,
										imu.gy,
										0,
										0,
										speed_pid.integral,
										u_turn,
										stm_pi_uart1_link.pid_update_available,
										pitch_angle_pid.Kp,
										pitch_angle_pid.Ki,
										pitch_angle_pid.Kd,
										speed_pid.Kp,
										speed_pid.Ki,
										speed_pid.Kd,
										0,
										controlQuantities.dt_actual,
										controlQuantities.dt_chassis);

	      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  //htim3.Init.Prescaler = 71; // 222.22[Hz]
  //htim3.Init.Prescaler = 7; // 20[kHz]
  htim3.Init.Prescaler = 31; // 5[kHz]

  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim3.Init.Period = 999;   // 222.22[Hz]
  //htim3.Init.Period = 99;  // 20[kHz]
  htim3.Init.Period = 100;  // 5[kHz]

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Redirects the printf output to UART
  */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
  * @brief  UART receive callback function.
  *         Handles incoming data frames on USART1, verifies CRC, and updates
  *         corresponding buffers or flags for further processing.
  * @param  huart: Pointer to the UART handle which triggered the callback.
  * @retval None
  * @note   The function supports multiple frame types: chassis data from the on board pi, PID tuning
  *         from a tuning station, and operator commands. It validates frames using CRC8 and flushes
  *         corrupted frames.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart1);

        uint8_t h1 = stm_pi_uart1_link.rx_buffer[0];
		uint8_t h2 = stm_pi_uart1_link.rx_buffer[1];
		uint8_t recv_crc = stm_pi_uart1_link.rx_buffer[PI_TO_STM_FRAME_TOTAL_SIZE-1];
		uint8_t calc_crc = crc8(stm_pi_uart1_link.rx_buffer, PI_TO_STM_FRAME_TOTAL_SIZE-1);

		if (recv_crc != calc_crc) {
			__HAL_UART_FLUSH_DRREGISTER(&huart1);
			__HAL_UART_CLEAR_OREFLAG(&huart1);
			HAL_UART_Transmit(&huart1, &stm_pi_uart1_link.ready_byte, 1, HAL_MAX_DELAY);
			goto relaunch_uart;
		}

		/* --- UART frame parsing --- */
		if (h1 == CHASSIS_FRAME_HEADER_1 && h2 == CHASSIS_FRAME_HEADER_2) {
			memcpy(stm_pi_uart1_link.chassis_buffer, stm_pi_uart1_link.rx_buffer, PI_TO_STM_FRAME_TOTAL_SIZE);
			stm_pi_uart1_link.chassis_update_available = 1;

		} else if (h1 == PID_TUNING_FRAME_HEADER_1 && h2 == PID_TUNING_FRAME_HEADER_2) {
			memcpy(stm_pi_uart1_link.pid_buffer, stm_pi_uart1_link.rx_buffer, PI_TO_STM_FRAME_TOTAL_SIZE);
			stm_pi_uart1_link.pid_update_available = 1;

		} else if (h1 == ROUTE1_FRAME_HEADER_1 && h2 == ROUTE1_FRAME_HEADER_2) {
			operator_commands.motorTestsIntervalCounter = 0;
			stm_pi_uart1_link.route1_activate = 1;

		} else if (h1 == ROUTE2_FRAME_HEADER_1 && h2 == ROUTE2_FRAME_HEADER_2) {
			stm_pi_uart1_link.route2_activate = 1;

		}
		/* --------------------------- */

relaunch_uart:

        // Re-arm UART1 interrupt
        HAL_UART_Receive_IT(&huart1, stm_pi_uart1_link.rx_buffer, PI_TO_STM_FRAME_TOTAL_SIZE);
    }
}


/**
  * @brief  Limits the change of a value to a maximum step.
  * @param  newValue: The target value to reach.
  * @param  oldValue: The current value from which the change is calculated.
  * @param  maxStep: Maximum allowed change (positive) per call.
  * @retval The clamped value, adjusted to not exceed maxStep from oldValue.
  */
/*
float clampSlew(float newValue, float oldValue, float maxStep)
{
    float delta = newValue - oldValue;

    if (delta > maxStep)
        return oldValue + maxStep;

    if (delta < -maxStep)
        return oldValue - maxStep;

    return newValue;
}
*/


uint8_t crc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

/**
  * @brief  UART transmit complete callback.
  *         Called by the HAL when a non-blocking UART transmission (HAL_UART_Transmit_IT)
  *         has completed.
  * @param  huart: Pointer to the UART handle that triggered the callback.
  * @retval None
  * @note   This function is required by the HAL when using HAL_UART_Transmit_IT().
  *         An empty callback must be implemented to avoid undefined behavior.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Nothing at the moment
    }
}


/**
  * @brief  Executes motor trial routines. Applies predefined PWM patterns to left and right motors for forward,
  *         backward, and idle intervals, including deadzone handling and saturation limits.
  *
  * @retval None
  *
  * @note   The function uses `operator_commands.motorTestsIntervalCounter` to determine
  *         the current phase of the test:
  *           - Even intervals <= 4: Constant forward PWM.
  *           - Even intervals 5-10: Constant backward PWM.
  *           - Odd intervals <= 10: Motors stopped.
  *           - Intervals > 10: Motors stopped.
  */
void do_MotorTrials(Motor_t* leftMotor, Motor_t* rightMotor) {

	if (stm_pi_uart1_link.route1_activate) {

		if (operator_commands.motorTestsIntervalCounter % 2 == 0 && operator_commands.motorTestsIntervalCounter <= 4) {

			// Const PWM, forward
			operator_commands.temp_target_PWM = 60.0; // Changes with PWM freq

			leftMotor->pwm = (int)do_turnWheel(HAL_GetTick(), 2300, 0, 0);
			rightMotor->pwm = (int)do_turnWheel(HAL_GetTick(), 2300, 0, 0);
			leftMotor->pwm  = apply_deadzone(leftMotor, leftMotor->pwm);
			rightMotor->pwm = apply_deadzone(rightMotor, rightMotor->pwm);

			leftMotor->pwm  = (leftMotor->pwm  >  MAX_EFFECTIVE_PWM) ?  MAX_EFFECTIVE_PWM :
						(leftMotor->pwm  < -MAX_EFFECTIVE_PWM) ? -MAX_EFFECTIVE_PWM : leftMotor->pwm;
			rightMotor->pwm = (rightMotor->pwm >  MAX_EFFECTIVE_PWM) ?  MAX_EFFECTIVE_PWM :
						(rightMotor->pwm < -MAX_EFFECTIVE_PWM) ? -MAX_EFFECTIVE_PWM : rightMotor->pwm;

			if( !(operator_commands.motorTestsDoOnceDummy == 2) ){
				setPWM(leftMotor, leftMotor->pwm);
				setPWM(rightMotor, rightMotor->pwm);
				operator_commands.motorTestsDoOnceDummy++;
			}

			stm_pi_uart1_link.route1_activate = 1;

		} else if (operator_commands.motorTestsIntervalCounter % 2 == 0 && operator_commands.motorTestsIntervalCounter > 4 && operator_commands.motorTestsIntervalCounter <= 10) {

			// Const PWM, backward
			operator_commands.temp_target_PWM = 30.0; // Changes with PWM freq

			leftMotor->pwm = (int)do_turnWheel(HAL_GetTick(), 2300, 0, 0);
			rightMotor->pwm = (int)do_turnWheel(HAL_GetTick(), 2300, 0, 0);
			leftMotor->pwm = -leftMotor->pwm;
			rightMotor->pwm = -rightMotor->pwm;
			leftMotor->pwm  = apply_deadzone(leftMotor, leftMotor->pwm);
			rightMotor->pwm = apply_deadzone(rightMotor, rightMotor->pwm);

			leftMotor->pwm  = (leftMotor->pwm  >  MAX_EFFECTIVE_PWM) ?  MAX_EFFECTIVE_PWM :
						(leftMotor->pwm  < -MAX_EFFECTIVE_PWM) ? -MAX_EFFECTIVE_PWM : leftMotor->pwm;
			rightMotor->pwm = (rightMotor->pwm >  MAX_EFFECTIVE_PWM) ?  MAX_EFFECTIVE_PWM :
						(rightMotor->pwm < -MAX_EFFECTIVE_PWM) ? -MAX_EFFECTIVE_PWM : rightMotor->pwm;

			if( !(operator_commands.motorTestsDoOnceDummy == 2) ){
				setPWM(leftMotor, leftMotor->pwm);
				setPWM(rightMotor, rightMotor->pwm);
				operator_commands.motorTestsDoOnceDummy++;
			}

			stm_pi_uart1_link.route1_activate = 1;

		} else if (operator_commands.motorTestsIntervalCounter % 2 == 1 && operator_commands.motorTestsIntervalCounter <= 10) {

			operator_commands.temp_target_PWM = 0;
			setPWM(leftMotor, (int)do_turnWheel(HAL_GetTick(), 7000, 0, 0));
			setPWM(rightMotor, (int)do_turnWheel(HAL_GetTick(), 7000, 0, 0));
			//setPWM_MotorLeft((int)do_turnWheel(HAL_GetTick(), 7000, 0, 0));
			//setPWM_MotorRight((int)do_turnWheel(HAL_GetTick(), 7000, 0, 0));
			stm_pi_uart1_link.route1_activate = 1;

		} else {

			setPWM(leftMotor, 0);
			setPWM(rightMotor, 0);
			stm_pi_uart1_link.route1_activate = 0;

		}
	}
}


/**
  * @brief  Checks for updated PID parameters received from the on-board pi over UART.
  * 		The raspberry pi listens on a socket to tunings from a station.
  *         If new parameters are available, the function updates the corresponding PID controllers
  *         and acknowledges receipt to the sender.
  * @param  None
  * @retval None
  */
void check_for_updates_from_tuning_setup() {
	if (stm_pi_uart1_link.pid_update_available == 1) {
		float *p = (float *)&stm_pi_uart1_link.pid_buffer[2];

		pitch_angle_pid.Kp = p[0];
		pitch_angle_pid.Ki = p[1];
		pitch_angle_pid.Kd = p[2];

		speed_pid.Kp = p[3];
		speed_pid.Ki = p[4];
		speed_pid.Kd = p[5];

		/* --- Be mindful of robot configuration --- */
//		leftMotor.Min_PWM_Forward = (int)p[6];
//		rightMotor.Min_PWM_Forward = (int)p[6];
//
//		leftMotor.Min_PWM_Backward = (int)p[7];
//		rightMotor.Min_PWM_Backward = (int)p[7];
//		leftMotor.Min_PWM_Backward = -leftMotor.Min_PWM_Backward;
//		rightMotor.Min_PWM_Backward = -rightMotor.Min_PWM_Backward;
		/* ---------------------------------------- */

		stm_pi_uart1_link.pid_update_available = 0;

		HAL_UART_Transmit_IT(&huart1, &stm_pi_uart1_link.ready_byte, 1);
	}
}


/**
  * @brief  Computes the control commands for forward and turning motions
  * @param  u_forward: Pointer to the output forward control command.
  * @param  u_turn: Pointer to the output turning (yaw) control command.
  * @retval None
  *
  * @note   The function performs the following steps:
  *           - Reads a single IMU sample for its gyros and acceleromoters values.
  *           - Updates a 1x1 Kalman filter for pitch estimation.
  *           - If new chassis data is available from UART (provided by the pi in interrupt mode):
  *               - Computes time delta since last chassis update.
  *               - Updates chassis speeds, xdot, psidot, and yaw angle.
  *               - Handles operator commands activation.
  *               - Updates speed PID controller based on target speed and measured xdot (wheel encoders).
  *           - Computes pitch angle control command using PID.
  *           - Combines pitch and speed controllers to produce forward control command.
  *           - Sets turning command based on yaw PID hold value.
  *
  * @note   The function uses `controlQuantities.dt_actual` for pitch PID and `controlQuantities.dt_chassis`
  *         for speed PID when new chassis data arrives.
  */
void compute_controllers_commands(float* u_forward, float* u_turn) {

	single_IMU_reading(&hspi1, &imu);
	float pitch_acc = atan2f(imu.ay, imu.az) * 57.2958f; // Signs explained in figure. basically - with accelerometers it's opposite of intuition
	float pitch_rate = imu.gx;
	Kalman1x1_Update(&pitch_kalman1x1, pitch_acc, pitch_rate, controlQuantities.dt_actual);
	//pitch_comp = alpha * (pitch_comp + pitch_rate * dt) + (1.0f - alpha) * pitch_acc; // Complementary filter to compare with. Outputs are similar


	/* -------- Slow control loop for wheel encoder data received from  the pi --------*/
	/* It can be seen from telemetry that is on average ~0.031[sec] (~30[Hz])*/
	if (stm_pi_uart1_link.chassis_update_available) {
		stm_pi_uart1_link.chassis_update_available = 0;

		/*------------- Compute dt_chassis -------------*/
		uint32_t now = HAL_GetTick();
		if (controlQuantities.last_chassis_tick != 0) {
			controlQuantities.dt_chassis =
				(now - controlQuantities.last_chassis_tick) * 0.001f;
		}
		controlQuantities.last_chassis_tick = now;
		/*----------------------------------------------*/

		/*------------- Update chassis data ------------*/
		float *data = (float*)&stm_pi_uart1_link.chassis_buffer[2]; // skip 2-byte header
		chassisData.left_wheel_speed_rad_sec  = data[0];
		chassisData.right_wheel_speed_rad_sec = data[1];
		chassisData.xdot                      = data[2];
		chassisData.psidot                    = data[3];
		chassisData.yaw_angle += chassisData.psidot * controlQuantities.dt_chassis * 57.2958f; // rad/sec to deg/sec
		/*----------------------------------------------*/

		/* -------------- Yaw Controller -------------- */
		/*
		controlQuantities.u_yaw_angle_hold = PID_Update(&yaw_angle_pid,
				controlQuantities.target_yaw_angle,
				chassisData.yaw_angle,
				chassisData.psidot,
				dt_chassis);
		*/
		/* -------------------------------------------- */

		HAL_UART_Transmit_IT(&huart1, &stm_pi_uart1_link.ready_byte, 1);

		/* -------- Operator commands handling -------- */
		if (stm_pi_uart1_link.route1_activate) {
			//controlQuantities.target_speed = do_route1(HAL_GetTick(), 1500, 650, 650);
			speed_pid.integral = 0;
			stm_pi_uart1_link.route1_activate = 0;
		}

		if (stm_pi_uart1_link.route2_activate) {
			controlQuantities.u_yaw_angle_hold = do_route2(HAL_GetTick(), 15000, 2000, 2000);
		}
		/* -------------------------------------------- */

		/* -------------- Speed Controller. No measured rate ---------- */
		controlQuantities.u_speed_hold = PID_Update(&speed_pid,
				controlQuantities.target_speed,
				chassisData.xdot,
				0.0f,
				controlQuantities.dt_chassis);

		controlQuantities.u_speed_hold  = (controlQuantities.u_speed_hold  >  speed_pid.output_limit) ?  speed_pid.output_limit :
						(controlQuantities.u_speed_hold  < -speed_pid.output_limit) ? -speed_pid.output_limit : controlQuantities.u_speed_hold;   //TODO: The limit belongs to controlQuantities, not to PID..
		/* ----------------------------------------------------------- */

	}
	/* --------------------------------------------------------------------------------*/

	/* -------- TODO: Yaw investigations --------*/
	/*
	float yaw_rate_world = gz * cosf(pitch_kalman1x1.state_estimate * (float)M_PI / 180.0f) - gy * sinf(pitch_kalman1x1.state_estimate * (float)M_PI / 180.0f);
	float yaw_rate_world = gz * cosf(pitch_kalman1x1.state_estimate * (float)M_PI / 180.0f) + gy * sinf(pitch_kalman1x1.state_estimate * (float)M_PI / 180.0f);
	float yaw_rate_world = sqrtf(gy * gy + gz * gz);
	Kalman1x1_Update(&yaw_kalman1x1, chassisData.psidot * (180.0f / M_PI), yaw_rate_world , dt);
	yaw_angle_kalman_rate_integration += yaw_kalman1x1.state_estimate * dt;
	yaw_angle_IMU_rate_integ_no_kalman += yaw_rate_world * dt;
	 */
	/* ------------------------------------------*/


	/* ------ Speed controller in Cascade ------ */
	/*
	float combined_angle_setpoint = (controlQuantities.target_pitch_angle + controlQuantities.u_speed_hold); // Used only when cascading the PID controllers. If they're added, this can be left out.
	controlQuantities.u_pitch_angle = PID_Update(&pitch_angle_pid,
			combined_angle_setpoint,
			pitch_kalman1x1.state_estimate,
			pitch_rate,
			dt);
	*/
	/* ------------------------------------------*/

	/* --- Speed controller in superposition --- */
	controlQuantities.u_pitch_angle = PID_Update(&pitch_angle_pid,
			controlQuantities.target_pitch_angle,
			pitch_kalman1x1.state_estimate,
			pitch_rate,
			controlQuantities.dt_actual);
	/* ------------------------------------------*/

	/* ----------- Limit controllers ----------- */
	controlQuantities.u_pitch_angle = (controlQuantities.u_pitch_angle >  pitch_angle_pid.output_limit) ?  pitch_angle_pid.output_limit :
					(controlQuantities.u_pitch_angle < -pitch_angle_pid.output_limit) ? -pitch_angle_pid.output_limit : controlQuantities.u_pitch_angle;  //TODO: The limit belongs to controlQuantities, not to PID..

	controlQuantities.u_yaw_angle_hold = (controlQuantities.u_yaw_angle_hold    >  yaw_angle_pid.output_limit) ?  yaw_angle_pid.output_limit :
					   (controlQuantities.u_yaw_angle_hold    < -yaw_angle_pid.output_limit) ? -yaw_angle_pid.output_limit : controlQuantities.u_yaw_angle_hold; //   //TODO: The limit belongs to controlQuantities, not to PID..
	/* ------------------------------------------*/


	/* --- Speed controller in Cascade --- */
	//float u_forward = controlQuantities.u_pitch_angle;

	/* --- Speed controller in superposition --- */
	*u_forward = controlQuantities.u_pitch_angle + controlQuantities.u_speed_hold;
	*u_turn    = controlQuantities.u_yaw_angle_hold;

}


/**
  * @brief  Sets the PWM commands for left and right motors to achieve balancing,
  *         based on forward and turning control inputs.
  * @retval None
  *
  * @note   The function performs the following steps:
  *           - Combines forward and turning commands to compute left and right PWM.
  *           - Applies motor deadzone adjustments.
  *           - Clamps PWM to `MAX_EFFECTIVE_PWM` which is the Auto Reload Register tim3 value.
  *           - Sends PWM commands to the motors
  */
void set_PWM_for_balancing(Motor_t* leftMotor, Motor_t* rightMotor, float u_forward, float u_turn) {

	leftMotor->pwm  = (int)(u_forward + u_turn); // Positive yaw angle requires positive u_turn
	rightMotor->pwm = (int)(u_forward - u_turn);

	leftMotor->pwm  = apply_deadzone(leftMotor, leftMotor->pwm);
	rightMotor->pwm = apply_deadzone(rightMotor, rightMotor->pwm);

	/* --- Optinal: limit maximum PWM incerement --- */
	/* TODO: Implement using Motor*
	pwm_left  = clampSlew(pwm_left,  last_pwm_left,  MAX_PWM_STEP);
	pwm_right = clampSlew(pwm_right, last_pwm_right, MAX_PWM_STEP);
	last_pwm_left = pwm_left;
	last_pwm_right = pwm_right;
	*/

	leftMotor->pwm  = (leftMotor->pwm  >  MAX_EFFECTIVE_PWM) ?  MAX_EFFECTIVE_PWM :
				(leftMotor->pwm  < -MAX_EFFECTIVE_PWM) ? -MAX_EFFECTIVE_PWM : leftMotor->pwm;
	rightMotor->pwm = (rightMotor->pwm >  MAX_EFFECTIVE_PWM) ?  MAX_EFFECTIVE_PWM :
				(rightMotor->pwm < -MAX_EFFECTIVE_PWM) ? -MAX_EFFECTIVE_PWM : rightMotor->pwm;

	setPWM(leftMotor, leftMotor->pwm);
	setPWM(rightMotor, rightMotor->pwm);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
