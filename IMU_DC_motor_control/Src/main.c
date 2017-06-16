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

TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// DC MACROS
#define DC_ENABLE HAL_GPIO_WritePin(DCEnable_GPIO_Port, DCEnable_Pin, GPIO_PIN_SET)
#define DC_DISABLE HAL_GPIO_WritePin(DCEnable_GPIO_Port, DCEnable_Pin, GPIO_PIN_RESET)
#define DC_TOGGLE HAL_GPIO_TogglePin(DCEnable_GPIO_Port, DCEnable_Pin)
uint16_t Duty = 0;

// ACCELEROMTER and MAGNETOMETER
#define LSM303_ACC_ADDRESS (0x19 << 1)
#define LSM303_ACC_CTRL_REG1_A 0x20
#define LSM303_ACC_CTRL_REG2_A 0x21
#define LSM303_ACC_CTRL_REG3_A 0x22
#define LSM303_ACC_CTRL_REG4_A 0x23
#define LSM303_ACC_CTRL_REG5_A 0x24
#define LSM303_ACC_CTRL_REG6_A 0x25
#define LSM303_ACC_SCALE 8.0
#define LSM303_MAG_ADDRESS (0x1E << 1)
#define LSM303_MAG_CRA_REG_M 0x00
#define LSM303_MAG_CRB_REG_M 0x01
#define LSM303_MAG_MR_REG_M 0x02

float Accel_g_x;
float Accel_g_y;
float Accel_g_z;

float Mag_Gauss_x;
float Mag_Gauss_y;
float Mag_Gauss_z;
//GYROSCOPE
#define L3GD20_REGISTER_CTRL_REG1 0x20   // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2 0x21   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3 0x22   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4 0x23   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG5 0x24   // 00000000   rw
#define L3GD20_GYRO_SENSITIVITY 0.00875
#define CS_GYRO_LOW HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0)
#define CS_GYRO_HIGH HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1)

#define RAD_TO_DEG 180/M_PI

//float Angle_x; //degrees
//float Angle_y; //degrees
//float Angle_z; //degrees

float pitch, roll, pitch_acc, roll_acc, yaw;
int16_t pitch_, roll_;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t readGyro(uint8_t address);
void sendSettings(uint8_t address, uint8_t settings);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Button_Pin) {
		DC_TOGGLE;
	}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t Data[6];
	int16_t Accel_raw_x;
	int16_t Accel_raw_y;
	int16_t Accel_raw_z;
//	int16_t Mag_raw_x;
//	int16_t Mag_raw_y;
//	int16_t Mag_raw_z;
//	int16_t Angular_velocity_x;
//	int16_t Angular_velocity_y;
//	int16_t Angular_velocity_z;
	uint8_t settings;
//	float dt = 0.0023; //not very good, need beter idea
//	float dt = 0.01; //time base to integrate
//	uint8_t Tx[50];
//	uint8_t size;
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
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3, (uint32_t*)&Duty, 1);
  //ACCELEROMETER CONFIGURATION
  	settings = 0b01010111; //100Hz, XYZ axis enable
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1,
  			&settings, 1, 50);

  	settings = 0b10000000; /*data from internal filter sent to output register and
  	FIFO, there is no gravitation vector, dunno why*/
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG2_A, 1,
  			&settings, 1, 50);

  	settings = 0b00000000; //everything disable
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG3_A, 1,
  			&settings, 1, 50);

  	settings = 0b00100000; //+/- 4G selection
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG4_A, 1,
  			&settings, 1, 50);

  	settings = 0b00000000; //nothing
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG5_A, 1,
  			&settings, 1, 50);

  	settings = 0b00000000; //nothing
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG6_A, 1,
  			&settings, 1, 50);
  /////////////////////////////////////////////////////////////////////////////////////////
  //MAGNETOMETER CONFIGURATION
  	settings = 0b00011000; //output rate 75Hz
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_MAG_ADDRESS, LSM303_MAG_CRA_REG_M, 1,
  			&settings, 1, 50);

  	settings = 0b00100000; //field range +/- 1.3 Gauss
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_MAG_ADDRESS, LSM303_MAG_CRB_REG_M, 1,
  			&settings, 1, 50);

  	settings = 0b00000000; //continuous-conversion mode
  	HAL_I2C_Mem_Write(&hi2c1, LSM303_MAG_ADDRESS, LSM303_MAG_MR_REG_M, 1,
  			&settings, 1, 50);
  /////////////////////////////////////////////////////////////////////////////////////////
  //GYROSCOPE CONFIGURATION
  	sendSettings(L3GD20_REGISTER_CTRL_REG1, 0b11001111); //output data rate, XYZ axis enable
  	sendSettings(L3GD20_REGISTER_CTRL_REG2, 0b00000000); //high pass filter normal mode and cutoff freq
  	sendSettings(L3GD20_REGISTER_CTRL_REG3, 0b00000000); //nothing
  	sendSettings(L3GD20_REGISTER_CTRL_REG4, 0b00000000); //continous update, 250 dps
  	sendSettings(L3GD20_REGISTER_CTRL_REG5, 0b00010000); //high pass filter enable
  	DC_ENABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//ACCELEROMETER
		HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, 0x28 | 0x80, 1, Data,
				6, 50);
		Accel_raw_x = ((Data[1] << 8) | Data[0]);
		Accel_raw_y = ((Data[3] << 8) | Data[2]);
		Accel_raw_z = ((Data[5] << 8) | Data[4]);

		Accel_g_x = (float) ((Accel_raw_x * LSM303_ACC_SCALE) / INT16_MAX);
		Accel_g_y = (float) ((Accel_raw_y * LSM303_ACC_SCALE) / INT16_MAX);
		Accel_g_z = (float) ((Accel_raw_z * LSM303_ACC_SCALE) / INT16_MAX);

		//MAGNETOMETER
//		HAL_I2C_Mem_Read(&hi2c1, LSM303_MAG_ADDRESS, 0x03 | 0x80, 1, Data,
//				6, 50);
//		Mag_raw_x = (Data[1] | (Data[0] << 8)); //X
//		Mag_raw_y = (Data[5] | (Data[4] << 8)); //Y
//		Mag_raw_z = (Data[3] | (Data[2] << 8)); //Z
//
//		Mag_Gauss_x = (float) Mag_raw_x / 1100;
//		Mag_Gauss_y = (float) Mag_raw_y / 1100;
//		Mag_Gauss_z = (float) Mag_raw_z / 980;

		//GYROSCOPE
//		Data[0] = readGyro(0x28);
//		Data[1] = readGyro(0x29);
//		Data[2] = readGyro(0x2A);
//		Data[3] = readGyro(0x2B);
//		Data[4] = readGyro(0x2C);
//		Data[5] = readGyro(0x2D);
//
//		Angular_velocity_x = (int16_t)((Data[1] << 8) | Data[0]);
//		Angular_velocity_y = (int16_t)((Data[3] << 8) | Data[2]);
//		Angular_velocity_z = (int16_t)((Data[5] << 8) | Data[4]);
//
//		Angular_velocity_x = (int16_t)((float)Angular_velocity_x * L3GD20_GYRO_SENSITIVITY); //dps
//		Angular_velocity_y = (int16_t)((float)Angular_velocity_y * L3GD20_GYRO_SENSITIVITY); //dps
//		Angular_velocity_z = (int16_t)((float)Angular_velocity_z * L3GD20_GYRO_SENSITIVITY); //dps

//		Angle_x += Angular_velocity_x*dt; //degrees
//		Angle_y += Angular_velocity_y*dt; //degrees
//		Angle_z += Angular_velocity_z*dt; //degrees

//		pitch += Angular_velocity_y*dt; //degress from gyro
//		roll += Angular_velocity_x*dt; //degress from gyro
//		yaw += Angular_velocity_z*dt;

//		if (fabsf(Accel_g_z - Accel_g_y) < 0.1) {
//			pitch_acc = atan2f(Accel_g_x, Accel_g_z)*RAD_TO_DEG; //around y from accelerometer
//			roll_acc = 0;
//		} else if (fabsf(Accel_g_z - Accel_g_x) < 0.1) {
//			pitch_acc = 0; //around y from accelerometer
//			roll_acc = atan2f(Accel_g_y, Accel_g_z)*RAD_TO_DEG; //around x from accelerometer
//		} else {
//			pitch_acc = atan2f(Accel_g_x, Accel_g_z)*RAD_TO_DEG; //around y from accelerometer
//			roll_acc = atan2f(Accel_g_y, Accel_g_z)*RAD_TO_DEG; //around x from accelerometer
//		}


//		pitch_acc = atan2f(Accel_g_x, Accel_g_z)*RAD_TO_DEG; //around y from accelerometer
//		roll_acc = atan2f(Accel_g_y, Accel_g_z)*RAD_TO_DEG; //around x from accelerometer
//
//		pitch = 0.98 * pitch + 0.02 * pitch_acc;
//		roll = 0.98 * roll + 0.02 * roll_acc;
//
//		if (fabsf(pitch - 90) < 10 || fabsf(pitch - (-90)) < 10) {
//			roll = 0;
//		}
//		if (fabsf(roll - 90) < 10 || fabsf(roll - (-90)) < 10) {
//			pitch = 0;
//		}
//		pitch_ = (int16_t)pitch;
//		roll_ = (int16_t)roll;

		Duty = 900*Accel_g_x + 200;
//		Duty = 180;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOE, CS_Pin|DCEnable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led1_Pin|led2_Pin|led3_Pin|led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin DCEnable_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DCEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin led2_Pin led3_Pin led4_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin|led3_Pin|led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t readGyro(uint8_t address) {
	CS_GYRO_LOW;
	address |= 0x80;
	uint8_t data;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &data, 1, 50);
	CS_GYRO_HIGH;
	return data;
}

void sendSettings(uint8_t address, uint8_t settings) {
	CS_GYRO_LOW;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
	CS_GYRO_HIGH;
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
