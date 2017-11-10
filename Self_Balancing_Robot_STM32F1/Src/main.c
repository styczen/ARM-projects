/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//DC motor macros
// Right motor
#define DC_IN1_RIGHT_ON HAL_GPIO_WritePin(DC_IN1_RIGHT_GPIO_Port, DC_IN1_RIGHT_Pin, 1)
#define DC_IN2_RIGHT_ON HAL_GPIO_WritePin(DC_IN2_RIGHT_GPIO_Port, DC_IN2_RIGHT_Pin, 1)
#define DC_IN1_RIGHT_OFF HAL_GPIO_WritePin(DC_IN1_RIGHT_GPIO_Port, DC_IN1_RIGHT_Pin, 0)
#define DC_IN2_RIGHT_OFF HAL_GPIO_WritePin(DC_IN2_RIGHT_GPIO_Port, DC_IN2_RIGHT_Pin, 0)
#define DC_ENABLE_RIGHT HAL_GPIO_WritePin(DC_ENABLE_RIGHT_GPIO_Port, DC_ENABLE_RIGHT_Pin, 1)
#define DC_DISABLE_RIGHT HAL_GPIO_WritePin(DC_ENABLE_RIGHT_GPIO_Port, DC_ENABLE_RIGHT_Pin, 0)
#define DC_DUTY_RIGHT TIM1->CCR1
// Left motor
#define DC_IN1_LEFT_ON HAL_GPIO_WritePin(DC_IN1_LEFT_GPIO_Port, DC_IN1_LEFT_Pin, 1)
#define DC_IN2_LEFT_ON HAL_GPIO_WritePin(DC_IN2_LEFT_GPIO_Port, DC_IN2_LEFT_Pin, 1)
#define DC_IN1_LEFT_OFF HAL_GPIO_WritePin(DC_IN1_LEFT_GPIO_Port, DC_IN1_LEFT_Pin, 0)
#define DC_IN2_LEFT_OFF HAL_GPIO_WritePin(DC_IN2_LEFT_GPIO_Port, DC_IN2_LEFT_Pin, 0)
#define DC_ENABLE_LEFT HAL_GPIO_WritePin(DC_ENABLE_LEFT_GPIO_Port, DC_ENABLE_LEFT_Pin, 1)
#define DC_DISABLE_LEFT HAL_GPIO_WritePin(DC_ENABLE_LEFT_GPIO_Port, DC_ENABLE_LEFT_Pin, 0)
#define DC_DUTY_LEFT TIM1->CCR4
// Both motors
#define DC_IN1_ON DC_IN1_RIGHT_ON; DC_IN1_LEFT_ON
#define DC_IN2_ON DC_IN2_RIGHT_ON; DC_IN2_LEFT_ON
#define DC_IN1_OFF DC_IN1_RIGHT_OFF; DC_IN1_LEFT_OFF
#define DC_IN2_OFF DC_IN2_RIGHT_OFF; DC_IN2_LEFT_OFF
#define DC_ENABLE DC_ENABLE_RIGHT; DC_ENABLE_LEFT
#define DC_DISABLE DC_DISABLE_RIGHT; DC_DISABLE_LEFT

uint8_t dc_shutdown;

// MPU6050 macros
#define MPU6050_ADDRESS 0xD0 // AD0 low; dont know why not 0x68 but 0xD0 works
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_I_AM 0x68
#define MPU6050_ACCEL_SENSITIVITY_SCALE ((float) 16384)
#define MPU6050_GYRO_SENSITIVITY_SCALE ((float) 131)

#define RAD_TO_DEG 180/M_PI
#define I2C_DEINIT {HAL_I2C_DeInit(&hi2c1); HAL_I2C_Init(&hi2c1);}

#define LED_ON HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 0)
#define LED_OFF HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 1)

uint8_t data[14];

float angle, set_point, error, drive, duty_stm_studio;

uint8_t data_uart[100], size = 0;

typedef struct {
	float dState; // Last position input

	float iState; // Integrator state
	float iMax, iMin; // Maximum and minimum allowable integrator state

	float pGain, // Proportional gain
			iGain, // Integrator gain
			dGain; // Derivative gain
} PID;
PID pid;

int32_t encoder;
volatile uint8_t cnt, key_lock;

uint16_t battery_adc;
float battery_voltage;

float pTerm, iTerm, dTerm;

uint8_t Received[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t battery_low(uint16_t battery_adc);
void HAL_SYSTICK_Callback(void) { /* 100kHz interrupt */
	static uint32_t counter = 0, battery_counter = 0;
	counter++;
	battery_counter++;

	// Encoder start
	if (!key_lock && !HAL_GPIO_ReadPin(switch_GPIO_Port, switch_Pin)) {
		key_lock = 1;

//		switch (cnt) {
//		case 0:
//			set_point = 0.01 * (TIM4->CNT / 2);
//			break;
//		case 1:
//			set_point = -0.01 * (TIM4->CNT / 2);
//			break;
//		default:
//			break;
//		}
//		cnt++;
//		if (cnt > 1)
//			cnt = 0;

		pid.dGain = 20*(TIM4->CNT / 2);
	} else if (key_lock && HAL_GPIO_ReadPin(switch_GPIO_Port, switch_Pin))
		key_lock = 0;
	// Encoder end

//	if (counter == 1000) { /* every 0.5 sec */
//		counter = 0;
//		size = sprintf((char*) data_uart, "P_GAIN: %d\tI_GAIN: %d\tD_GAIN: %d\n\r", (int) (pid.pGain), (int) (pid.iGain), (int) (pid.dGain));
//		HAL_UART_Transmit_IT(&huart2, data_uart, size);
//	}
//	\tI: %d\tD: %d\tenc: %d\tcnt: %d

	// Battery voltage warning
//	if (battery_counter == 5000) { /* every 5 sec */
//		battery_counter = 0;
//		if (battery_low(battery_adc))
//			LED_ON;
//		else
//			LED_OFF;
//	}
}


void MPU6050_config(void);
float MPU6050_calibration(void);
float estimate_angle(void);
float update_PID(PID * pid, float error, float position);
void motor_plant(float drive, float position);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	pid.pGain = 300;
	pid.iGain = 0.05;
	pid.dGain = 0;

	pid.iMax = 1000;
	pid.iMin = -1000;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	MPU6050_config();
//	set_point = MPU6050_calibration();
	set_point = -10.00;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &battery_adc, 1);

	TIM4->CNT = 400;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		angle = estimate_angle();
		error = set_point - angle;
		drive = update_PID(&pid, set_point - angle, angle);
		motor_plant(drive, angle);

		//***********************
		encoder = TIM4->CNT / 2;
		battery_voltage = battery_adc / 4095.0 * 12.0 + 2.0;
		//***********************

//		DC_IN1_LEFT_ON;
//		DC_IN2_LEFT_OFF;
//		DC_DUTY_LEFT = 400;
//
//		DC_IN1_RIGHT_OFF;
//		DC_IN2_RIGHT_ON;
//		DC_DUTY_RIGHT = 1000;
//
//		HAL_Delay(1500);
//
//		DC_IN1_LEFT_OFF;
//		DC_IN2_LEFT_ON;
//		DC_DUTY_LEFT = 1000;
//
//		DC_IN1_RIGHT_ON;
//		DC_IN2_RIGHT_OFF;
//		DC_DUTY_RIGHT = 400;
//
//		HAL_Delay(1500);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 359;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 2200;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			DC_IN1_LEFT_Pin | DC_IN2_LEFT_Pin | DC_IN1_RIGHT_Pin
					| DC_IN2_RIGHT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : led_Pin */
	GPIO_InitStruct.Pin = led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DC_IN1_LEFT_Pin DC_IN2_LEFT_Pin DC_IN1_RIGHT_Pin DC_IN2_RIGHT_Pin */
	GPIO_InitStruct.Pin = DC_IN1_LEFT_Pin | DC_IN2_LEFT_Pin | DC_IN1_RIGHT_Pin
			| DC_IN2_RIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : switch_Pin */
	GPIO_InitStruct.Pin = switch_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(switch_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MPU6050_config(void) {
	LED_ON;
	uint8_t settings = 0;
	uint8_t temp = 0;

	while (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 100, 10) != HAL_OK)
		; // I2C_DEINIT
	while (temp != MPU6050_I_AM) {
		HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_WHO_AM_I, 1, &temp,
				1);
	}
	// Power config
	settings = 0x00;
	while (HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 1,
			&settings, 1) != HAL_OK)
		;

	// Gyro config
	settings = 0x00;
//	while (HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1, &temp, 1)!=HAL_OK);
//	settings = (temp & 0xE7) | settings;
	while (HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1,
			&settings, 1) != HAL_OK)
		;

	// Accel config
	settings = 0x00;
//	while (HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 1, &temp, 1)!=HAL_OK);
//	settings = (temp & 0xE7) | settings;
	while (HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG,
			1, &settings, 1) != HAL_OK)
		;

	LED_OFF;
}

float MPU6050_calibration(void) {
	HAL_Delay(2000);
//	uint8_t temp=0;
//	while (temp != MPU6050_I_AM) {
//		HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_WHO_AM_I, 1, &temp, 1);
//	}
	float angle_bias = 0;
	for (uint16_t i = 0; i < 1000; i++) {
		angle_bias += estimate_angle();
		HAL_Delay(1);
	}
	angle_bias /= 1000.0;
	return angle_bias;
}

float estimate_angle(void) {
	uint32_t currTimeStamp, elapsedTime;
	static uint32_t prevTimeStamp;
	float current_angle;
	static float prev_angle = 0;

	currTimeStamp = HAL_GetTick();
	elapsedTime = currTimeStamp - prevTimeStamp;
	prevTimeStamp = currTimeStamp;

	HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 1, data,
			14);

	// Accel
	int16_t accel_raw_x = ((data[0] << 8) | data[2]);
//	int16_t accel_raw_y = ((data[2] << 8) | data[3]);
	int16_t accel_raw_z = ((data[4] << 8) | data[5]);

	float accel_x = (float) accel_raw_x / MPU6050_ACCEL_SENSITIVITY_SCALE;
//	float accel_y = (float) accel_raw_y / MPU6050_ACCEL_SENSITIVITY_SCALE;
	float accel_z = (float) accel_raw_z / MPU6050_ACCEL_SENSITIVITY_SCALE;

	float angle_accel = atan2f(accel_x, accel_z) * RAD_TO_DEG; //only accel

	// Gyro
//	int16_t gyro_raw_x = ((data[8] << 8) | data[9]);
	int16_t gyro_raw_y = ((data[10] << 8) | data[11]);

	float gyro_y = (float) gyro_raw_y / MPU6050_GYRO_SENSITIVITY_SCALE;

	// Complementary filter
	current_angle = 0.997
			* (prev_angle + (gyro_y * (float) elapsedTime / 1000.0))
			+ 0.003 * angle_accel;
	prev_angle = current_angle;

	return current_angle;
}

float update_PID(PID * pid, float error, float position) {
//	float pTerm, iTerm, dTerm;
	// P
	pTerm = pid->pGain * error;

	// I
	pid->iState += error;
	if (pid->iState > pid->iMax)
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin)
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState;

	// D
	dTerm = pid->dGain * (pid->dState - position);
	pid->dState = position;

	return pTerm + iTerm + dTerm;
}

void motor_plant(float drive, float position) {
	if (position > 25 || position < -25 || position == 0) {
		DC_DISABLE
		;
		DC_DUTY_RIGHT = 0;
		DC_DUTY_LEFT = 0;
	} else if (drive > 0) {
		if (drive > 1000)
			drive = 1000;
		DC_ENABLE
		;
		DC_DUTY_RIGHT = drive;
		DC_DUTY_LEFT = drive;
		//*****
		duty_stm_studio = drive;
		//*****
		DC_IN2_OFF
		;
		DC_IN1_ON
		;
	} else if (drive < 0) {
		if (drive < -1000)
			drive = -1000;
		DC_ENABLE
		;
		DC_DUTY_RIGHT = -drive;
		DC_DUTY_LEFT = -drive;
		//*****
		duty_stm_studio = drive;
		//*****
		DC_IN1_OFF
		;
		DC_IN2_ON
		;
	} else {
		DC_DISABLE
		;
	}
}

uint8_t battery_low(uint16_t battery_adc) {
	float battery_voltage = (float) battery_adc / 4095.0 * 12.0 + 2.0; // dont care about resistors
	if (battery_voltage < 10)
		return 1;
	return 0;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
