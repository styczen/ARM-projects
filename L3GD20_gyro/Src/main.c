/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
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
#include <limits.h>
#include <time.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define L3GD20_WHO_AM_I				         0xD4
#define L3GD20_REGISTER_WHO_AM_I             0x0F   // 11010100   r
#define L3GD20_REGISTER_CTRL_REG1            0x20   // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2            0x21   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3            0x22   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4            0x23   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG5            0x24   // 00000000   rw
#define L3GD20_REGISTER_REFERENCE            0x25   // 00000000   rw
#define L3GD20_REGISTER_OUT_TEMP             0x26   //            r
#define L3GD20_REGISTER_STATUS_REG           0x27   //            r
#define L3GD20_REGISTER_OUT_X_L              0x28   //            r
#define L3GD20_REGISTER_OUT_X_H              0x29   //            r
#define L3GD20_REGISTER_OUT_Y_L              0x2A   //            r
#define L3GD20_REGISTER_OUT_Y_H              0x2B   //            r
#define L3GD20_REGISTER_OUT_Z_L              0x2C   //            r
#define L3GD20_REGISTER_OUT_Z_H              0x2D   //            r
#define L3GD20_REGISTER_FIFO_CTRL_REG        0x2E   // 00000000   rw
#define L3GD20_REGISTER_FIFO_SRC_REG         0x2F   //            r
#define L3GD20_REGISTER_INT1_CFG             0x30   // 00000000   rw
#define L3GD20_REGISTER_INT1_SRC             0x31   //            r
#define L3GD20_REGISTER_TSH_XH               0x32   // 00000000   rw
#define L3GD20_REGISTER_TSH_XL               0x33   // 00000000   rw
#define L3GD20_REGISTER_TSH_YH               0x34   // 00000000   rw
#define L3GD20_REGISTER_TSH_YL               0x35   // 00000000   rw
#define L3GD20_REGISTER_TSH_ZH               0x36   // 00000000   rw
#define L3GD20_REGISTER_TSH_ZL               0x37   // 00000000   rw
#define L3GD20_REGISTER_INT1_DURATION        0x38   // 00000000   rw

#define L3GD20_SENSITIVITY_250		0.00875	/* 8.75 mdps/digit */
#define L3GD20_SENSITIVITY_500		0.0175	/* 17.5 mdps/digit */
#define L3GD20_SENSITIVITY_2000		0.070	/* 70 mdps/digit */
//#define GYRO_FS_DPS 250

#define CS_LOW HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0)
#define CS_HIGH HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1)

uint8_t settings, address;
uint8_t data[6];
int16_t angular_velocity_x;
int16_t angular_velocity_y;
int16_t angular_velocity_z;
float angle_x; //degrees
float angle_y; //degrees
float angle_z; //degrees
float temp, s;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t readGyro(uint8_t address);
void sendSettings(uint8_t address, uint8_t settings);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	float dt = 0.000043;
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();

	/* USER CODE BEGIN 2 */

	// Settings - begin
	sendSettings(L3GD20_REGISTER_CTRL_REG1, 0x0F);
	sendSettings(L3GD20_REGISTER_CTRL_REG2, 0x00);
	sendSettings(L3GD20_REGISTER_CTRL_REG3, 0x00);
	sendSettings(L3GD20_REGISTER_CTRL_REG4, 0x00);
	sendSettings(L3GD20_REGISTER_CTRL_REG5, 0x10);

//	CS_LOW;
//	address = L3GD20_REGISTER_CTRL_REG1;
//	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//	settings = 0x0F;
//	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
//	CS_HIGH;
//
//	CS_LOW;
//	address = L3GD20_REGISTER_CTRL_REG2;
//	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//	settings = 0x00;
//	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
//	CS_HIGH;
//
//	CS_LOW;
//	address = L3GD20_REGISTER_CTRL_REG3;
//	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//	settings = 0x00;
//	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
//	CS_HIGH;
//
//	CS_LOW;
//	address = L3GD20_REGISTER_CTRL_REG4;
//	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//	settings = 0x00;
//	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
//	CS_HIGH;
//
//	CS_LOW;
//	address = L3GD20_REGISTER_CTRL_REG5;
//	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//	settings = 0x10;
//	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
//	CS_HIGH;
	// Settings - end
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		data[0] = readGyro(L3GD20_REGISTER_OUT_X_L);
		data[1] = readGyro(L3GD20_REGISTER_OUT_X_H);
		data[2] = readGyro(L3GD20_REGISTER_OUT_Y_L);
		data[3] = readGyro(L3GD20_REGISTER_OUT_Y_H);
		data[4] = readGyro(L3GD20_REGISTER_OUT_Z_L);
		data[5] = readGyro(L3GD20_REGISTER_OUT_Z_H);

//		CS_LOW;
//		address = L3GD20_REGISTER_OUT_X_L | 0x80;
//		HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//		HAL_SPI_Receive(&hspi1, &data[0], 1, 50);
//		CS_HIGH;
//
//		CS_LOW;
//		address = L3GD20_REGISTER_OUT_X_H | 0x80;
//		HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//		HAL_SPI_Receive(&hspi1, &data[1], 1, 50);
//		CS_HIGH;
//
//		CS_LOW;
//		address = L3GD20_REGISTER_OUT_Y_L | 0x80;
//		HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//		HAL_SPI_Receive(&hspi1, &data[2], 1, 50);
//		CS_HIGH;
//
//		CS_LOW;
//		address = L3GD20_REGISTER_OUT_Y_H | 0x80;
//		HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//		HAL_SPI_Receive(&hspi1, &data[3], 1, 50);
//		CS_HIGH;
//
//		CS_LOW;
//		address = L3GD20_REGISTER_OUT_Z_L | 0x80;
//		HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//		HAL_SPI_Receive(&hspi1, &data[4], 1, 50);
//		CS_HIGH;
//
//		CS_LOW;
//		address = L3GD20_REGISTER_OUT_Z_H | 0x80;
//		HAL_SPI_Transmit(&hspi1, &address, 1, 50);
//		HAL_SPI_Receive(&hspi1, &data[5], 1, 50);
//		CS_HIGH;

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		HAL_Delay(200);
		angular_velocity_x = (int16_t)((data[1] << 8) | data[0]); // x
		angular_velocity_y = (int16_t)((data[3] << 8) | data[2]); // y
		angular_velocity_z = (int16_t)((data[5] << 8) | data[4]); // z

		s = L3GD20_SENSITIVITY_250;

		temp = (float)angular_velocity_x * s;
		angular_velocity_x = (int16_t) (temp);

		temp = (float)angular_velocity_y * s;
		angular_velocity_y = (int16_t) (temp);

		temp = (float)angular_velocity_z * s;
		angular_velocity_z = (int16_t) (temp);

		angle_x += angular_velocity_x*dt;
		angle_y += angular_velocity_y*dt;
		angle_z += angular_velocity_z*dt;

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

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
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

/* SPI1 init function */
static void MX_SPI1_Init(void) {

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}

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
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t readGyro(uint8_t address) {
	CS_LOW;
	address |= 0x80;
	uint8_t data;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &data, 1, 50);
	CS_HIGH;
	return data;
}

void sendSettings(uint8_t address, uint8_t settings) {
	CS_LOW;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
	CS_HIGH;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
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
