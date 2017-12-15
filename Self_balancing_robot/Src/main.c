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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// ACCELEROMTER
#define LSM303_ACC_ADDRESS (0x19 << 1)
#define LSM303_ACC_CTRL_REG1_A 0x20
#define LSM303_ACC_CTRL_REG2_A 0x21
#define LSM303_ACC_CTRL_REG3_A 0x22
#define LSM303_ACC_CTRL_REG4_A 0x23
#define LSM303_ACC_CTRL_REG5_A 0x24
#define LSM303_ACC_CTRL_REG6_A 0x25
#define LSM303_ACC_SCALE 2.0f

//GYROSCOPE
#define L3GD20_REGISTER_CTRL_REG1 0x20   // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2 0x21   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3 0x22   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4 0x23   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG5 0x24   // 00000000   rw
#define L3GD20_GYRO_SENSITIVITY 0.00875f

#define RAD_TO_DEG 180/M_PI

//Gyro macros
#define CS_GYRO_LOW HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, 0)
#define CS_GYRO_HIGH HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, 1)
//DC motor macros
#define DC_IN1_ON HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, 1)
#define DC_IN2_ON HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, 1)
#define DC_IN1_OFF HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, 0)
#define DC_IN2_OFF HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, 0)
#define DC_ENABLE HAL_GPIO_WritePin(DC_EN_GPIO_Port, DC_EN_Pin, 1)
#define DC_DISABLE HAL_GPIO_WritePin(DC_EN_GPIO_Port, DC_EN_Pin, 0)
#define DC_TOGGLE HAL_GPIO_TogglePin(DC_EN_GPIO_Port, DC_EN_Pin)
uint8_t dc_shutdown;
//Other variables
float gyro_bias;

float pitch, accel_angle;
//float accel_angle, current_angle, prev_angle;

uint8_t settings = 0;

//int16_t Accel_raw_x, Accel_raw_y, Accel_raw_z;
//float Accel_g_x, Accel_g_y, Accel_g_z, angle;

uint8_t Data[6];

//Elapsed time variables
//float currTimeStamp=0, elapsedTime=0, prevTimeStamp=0;
typedef struct {
	float dState; // Last position input
	float iState; // Integrator state
	float iMax, iMin; // Maximum and minimum allowable integrator state
	float pGain, // Proportional gain
	      iGain, // Integrator gain
		  dGain; // Derivative gain
} PID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t read_gyro(uint8_t address);
void send_settings(uint8_t address, uint8_t settings);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Button_Pin) {
		dc_shutdown ^= 1;
	}
}
void accel_config(void);
void gyro_config(void);
void HAL_SYSTICK_Callback(void) {
	static int cnt = 0;
	cnt++;
	if (cnt == 1000) {
		cnt = 0;
		HAL_GPIO_TogglePin(LED_green_GPIO_Port, LED_green_Pin);
	}
}
float get_accel_angle();
float estimate_angle();
float gyro_calibration();
float update_PID(PID * pid, float error, float set_point);
/* USEr CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

	//PWM start
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	//ACCELEROMETER CONFIGURATION
	accel_config();
	//GYROSCOPE CONFIGURATION
	gyro_config();
	gyro_bias = gyro_calibration();

	DC_ENABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (!dc_shutdown) {
//			TIM1->CCR3 = 0;
//			accel_angle = get_accel_angle();
			pitch = estimate_angle();
			if (pitch > -15 && pitch < 0) {
				TIM1->CCR3 = -pitch * 1500;
				DC_IN1_ON;
				DC_IN2_OFF;
			} else if (pitch < 15 && pitch > 0) {
				TIM1->CCR3 = pitch * 1500;
				DC_IN1_OFF;
				DC_IN2_ON;
			}
		} else {
			DC_DISABLE;
		}
//		DC_IN1_ON;
//		TIM1->CCR3 = 4000;
//		DC_IN1_ON;
//		TIM1->CCR3 = 1000;


		//PID

//		HAL_Delay(10);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_green_Pin|LED_orange_Pin|LED_red_Pin|LED_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DC_IN1_Pin|DC_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_GYRO_Pin */
  GPIO_InitStruct.Pin = CS_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GYRO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_green_Pin LED_orange_Pin LED_red_Pin LED_blue_Pin */
  GPIO_InitStruct.Pin = LED_green_Pin|LED_orange_Pin|LED_red_Pin|LED_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_IN1_Pin DC_IN2_Pin */
  GPIO_InitStruct.Pin = DC_IN1_Pin|DC_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t read_gyro(uint8_t address) {
	CS_GYRO_LOW;
	address |= 0x80;
	uint8_t data;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &data, 1, 50);
	CS_GYRO_HIGH;
	return data;
}

void send_settings(uint8_t address, uint8_t settings) {
	CS_GYRO_LOW;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
	CS_GYRO_HIGH;
}

void accel_config(void) {
	settings = 0b01010111; //100Hz, XYZ axis enable
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1,
			&settings, 1, 50);

	settings = 0b10000000; //data from internal filter sent to output register and FIFO, there is no gravitation vector, dunno why
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG2_A, 1,
			&settings, 1, 50);

	settings = 0b00000000; //everything disable
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG3_A, 1,
			&settings, 1, 50);

	settings = 0b00000000; // +/- 2G selection
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG4_A, 1,
			&settings, 1, 50);

	settings = 0b00000000; //nothing
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG5_A, 1,
			&settings, 1, 50);

	settings = 0b00000000; //nothing
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG6_A, 1,
			&settings, 1, 50);
}

void gyro_config(void) {
	send_settings(L3GD20_REGISTER_CTRL_REG1, 0b11001111); //output data rate, XYZ axis enable
	send_settings(L3GD20_REGISTER_CTRL_REG2, 0b00000000); //high pass filter normal mode and cutoff freq
	send_settings(L3GD20_REGISTER_CTRL_REG3, 0b00000000); //nothing
	send_settings(L3GD20_REGISTER_CTRL_REG4, 0b00000000); //continous update, 250 dps
	send_settings(L3GD20_REGISTER_CTRL_REG5, 0b00010000); //high pass filter enable
}

float get_accel_angle() {
	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, 0x28 | 0x80, 1, Data,
			6, 50);
	int16_t Accel_raw_x = ((Data[1] << 8) | Data[0]);
//	int16_t Accel_raw_y = ((Data[3] << 8) | Data[2]);
	int16_t Accel_raw_z = ((Data[5] << 8) | Data[4]);

	float Accel_g_x = (float) ((Accel_raw_x * LSM303_ACC_SCALE) / (float)INT16_MAX);
//	float Accel_g_y = (float) ((Accel_raw_y * LSM303_ACC_SCALE) / (float)INT16_MAX);
	float Accel_g_z = (float) ((Accel_raw_z * LSM303_ACC_SCALE) / (float)INT16_MAX);

	float angle = atan2f(Accel_g_x, Accel_g_z)*RAD_TO_DEG;
	return angle;
}

float estimate_angle() {
	uint32_t currTimeStamp, elapsedTime;
	static uint32_t prevTimeStamp;
	float current_angle;
	static float prev_angle=0;

    currTimeStamp = HAL_GetTick();
    elapsedTime = currTimeStamp - prevTimeStamp;
    prevTimeStamp = currTimeStamp;

//	Data[0] = read_gyro(0x28);
//	Data[1] = read_gyro(0x29);
	Data[2] = read_gyro(0x2A);
	Data[3] = read_gyro(0x2B);
//	Data[4] = read_gyro(0x2C);
//	Data[5] = read_gyro(0x2D);

//	int16_t Angular_velocity_x_raw = (int16_t)((Data[1] << 8) | Data[0]);
	int16_t Angular_velocity_y_raw = (int16_t)((Data[3] << 8) | Data[2]);
//	int16_t Angular_velocity_z_raw = (int16_t)((Data[5] << 8) | Data[4]);

//	float Angular_velocity_x = (float)Angular_velocity_x_raw * L3GD20_GYRO_SENSITIVITY; //dps
	float Angular_velocity_y = (float)Angular_velocity_y_raw * L3GD20_GYRO_SENSITIVITY; //dps
//	float Angular_velocity_z = (float)Angular_velocity_z_raw * L3GD20_GYRO_SENSITIVITY; //dps

	Angular_velocity_y -= gyro_bias;
	//Complementary filter
	current_angle = 0.997 * (prev_angle + (Angular_velocity_y * (float)elapsedTime/1000.0)) + 0.003 * get_accel_angle();
	prev_angle = current_angle;

	return current_angle;
}

float gyro_calibration() {
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, 1);
	uint8_t data[2];
	int16_t angular_raw;
	float bias_value=0, angular_velocity;
	HAL_Delay(1500);
	for (uint8_t i = 0; i < 100; i++) {
		data[0] = read_gyro(0x2A);
		data[1] = read_gyro(0x2B);
		angular_raw = (int16_t)((data[1] << 8) | data[0]);
		angular_velocity = (float)angular_raw * L3GD20_GYRO_SENSITIVITY; //dps
		bias_value += angular_velocity;
	}
	bias_value /= 100.0;
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, 0);
	return bias_value;
}

float update_PID(PID * pid, float error, float set_point) {
	float pTerm, iTerm, dTerm;
	// P
	pTerm = pid->pGain * error;
	// I
	pid->iState += error;
	if (pid->iState > pid->iMax) pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState;
	// D
	dTerm = pid->dGain * (pid->dState - set_point);
	pid->dState = set_point;

	return pTerm + iTerm + dTerm;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
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
