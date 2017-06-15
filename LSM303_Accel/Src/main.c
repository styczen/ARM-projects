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

/* USER CODE BEGIN Includes */
#include <limits.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//Macrodefinitions
#define LSM303_ACC_ADDRESS (0x19 << 1) //shift to left because LSB can be 1 or 0 depending on Read/Write operation; HAL handles it by itself
//CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPen][Zen][Yen][Xen]
#define LSM303_ACC_CTRL_REG1_A 0x20
#define LSM303_ACC_POWER_DOWN 0x00
#define LSM303_ACC_1HZ 0x10
#define LSM303_ACC_10HZ 0x20
#define LSM303_ACC_25HZ 0x30
#define LSM303_ACC_50HZ 0x40
#define LSM303_ACC_100HZ 0x50
#define LSM303_ACC_200HZ 0x60
#define LSM303_ACC_400HZ 0x70
#define LSM303_ACC_1620HZ 0x80 //onlu Low-power mode
#define LSM303_ACC_1344HZ_NORMAL_OR_5376HZ_LOW 0x90
#define LSM303_ACC_XYZ_ENABLE 0x07
#define LSM303_ACC_X_ENABLE 0x01
#define LSM303_ACC_Y_ENABLE 0x02
#define LSM303_ACC_Z_ENABLE 0x04
#define LSM303_ACC_LOW_POWER 0x08

//CTRL_REG2_A = [HPM1][HPM0][HPCF2][HPCF1][FDS][HPCLICK][HPIS2][HPIS1]
#define LSM303_ACC_CTRL_REG2_A 0x21
//Some high pass filter options.

//CTRL_REG3_A = [I1_CLICK][I1_AOI1][[I1_AOI2][I1_DRDY1][I1_DRDY2][I1_WTM][I1_OVERRUN][--]
#define LSM303_ACC_CTRL_REG3_A 0x22
//Some INT1 and FIFO options.

//CTRL_REG4_A = [BDU][BLE][FS1][FS0][HR][0][0][SIM]
#define LSM303_ACC_CTRL_REG4_A 0x23
#define LSM303_ACC_HIGH_RES_OUT 0x08
#define LSM303_ACC_2G 0x00
#define LSM303_ACC_4G 0x10
#define LSM303_ACC_8G 0x20
#define LSM303_ACC_16G 0x30
#define LSM303_ACC_SPI_3_WIRE 0x01 //otherwise 4-wire SPI

//CTRL_REG5_A = [BOOT][FIFO_EN][--][--][LIR_INT1][D4D_INT1][LIR_INT2][D4D_INT2]
#define LSM303_ACC_CTRL_REG5_A 0x24
#define LSM303_ACC_REBOOT_MEMORY 0x80
#define LSM303_ACC_FIFO_ENABLE 0x40
//Some Latch interrput and 4D (?) enable.

//CTRL_REG6_A = [I2_CLICKen][I2_INT1][I2_INT2][BOOT_I1][P2_ACT][--][H_LACTIVE][--]
#define LSM303_ACC_CTRL_REG6_A 0x25
//Some interrupts and reboot

//REFERENCE/DATACAPTURE_A = [Ref7][Ref6][Ref5][Ref4][Ref3][Ref2][Ref1][Ref0]
#define LSM303_ACC_REFERENCE_DATACAPTURE_A 0x26

//STATUS_REG_A = [ZYXOR][ZOR][YOR][XOR][ZYXDA][ZDA][YDA][XDA]
#define LSM303_ACC_STATUS_REG_A 0x27
//Some data overrun and...
#define LSM303_ACC_ZYXDA 0x08 //X, Y, Z axis new data avaiable
#define LSM303_ACC_ZDA 0x04 //Z axis new data avaiable
#define LSM303_ACC_YDA 0x02 //Y axis new data avaiable
#define LSM303_ACC_XDA 0x01 //X axis new data avaiable

//OUT REGISTERS
#define LSM303_ACC_OUT_X_L_A 0x28
#define LSM303_ACC_OUT_X_H_A 0x29
#define LSM303_ACC_OUT_Y_L_A 0x2A
#define LSM303_ACC_OUT_Y_H_A 0x2B
#define LSM303_ACC_OUT_Z_L_A 0x2C
#define LSM303_ACC_OUT_Z_H_A 0x2D
#define LSM303_ACC_OUT_Z_MULTIREAD_A (LSM303_ACC_OUT_Z_L_A | 0x80)
#define LSM303_ACC_OUT_XYZ_MULTIREAD_A (LSM303_ACC_OUT_X_L_A | 0x80) //0x80 from manual to enable read from 2 bytes

#define LSM303_ACC_RESOLUTION_2G 2.0
#define LSM303_ACC_RESOLUTION_4G 4.0
#define LSM303_ACC_RESOLUTION_8G 8.0
#define LSM303_ACC_RESOLUTION_16G 16.0
//Variables
uint8_t Data[6];
int16_t Accel_raw_x;
int16_t Accel_raw_y;
int16_t Accel_raw_z;
float Accel_g_x;
float Accel_g_y;
float Accel_g_z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t settings;
	float Threshold = 0.4;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  //********************************************************************************************************************
 	//Using blocking mode.
   	settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;
 	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &settings, 1, 100);

 	settings = LSM303_ACC_8G;
 	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG4_A, 1, &settings, 1, 100);
   //********************************************************************************************************************

 	//Using DMA but do not working; don't know why...
 //  settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;
 //  HAL_I2C_Mem_Write_DMA(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &settings, 1);
 //
 //  settings = LSM303_ACC_2G;
 //  HAL_I2C_Mem_Write_DMA(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG4_A, 1, &settings, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//********************************************************************************************************************
	//		HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_OUT_XYZ_MULTIREAD_A, 1, Data, 6, 100);
			//********************************************************************************************************************
			HAL_I2C_Mem_Read_DMA(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_OUT_XYZ_MULTIREAD_A, 1, Data, 6);
			Accel_raw_x = ((Data[1] << 8) | Data[0]);
			Accel_raw_y = ((Data[3] << 8) | Data[2]);
			Accel_raw_z = ((Data[5] << 8) | Data[4]);

			Accel_g_x = (float)((Accel_raw_x * LSM303_ACC_RESOLUTION_8G)/INT16_MAX);
			Accel_g_y = (float)((Accel_raw_y * LSM303_ACC_RESOLUTION_8G)/INT16_MAX);
			Accel_g_z = (float)((Accel_raw_z * LSM303_ACC_RESOLUTION_8G)/INT16_MAX);


			//Y axis
			if (Accel_g_x > Threshold) {
				HAL_GPIO_WritePin(Y_POSITIVE_GPIO_Port, Y_POSITIVE_Pin, GPIO_PIN_SET);
			} else if (Accel_g_x < -Threshold) {
				HAL_GPIO_WritePin(Y_NEGATIVE_GPIO_Port, Y_NEGATIVE_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOD, Y_POSITIVE_Pin | Y_NEGATIVE_Pin, GPIO_PIN_RESET);
			}
			//X axis
			if (Accel_g_y < -Threshold) {
				HAL_GPIO_WritePin(X_POSITIVE_GPIO_Port, X_POSITIVE_Pin, GPIO_PIN_SET);
			} else if (Accel_g_y > Threshold) {
				HAL_GPIO_WritePin(X_NEGATIVE_GPIO_Port, X_NEGATIVE_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOD, X_POSITIVE_Pin | X_NEGATIVE_Pin, GPIO_PIN_RESET);
			}

	//		HAL_GPIO_WritePin(X_POSITIVE_GPIO_Port, X_POSITIVE_Pin, GPIO_PIN_SET);

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
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, X_NEGATIVE_Pin|Y_POSITIVE_Pin|X_POSITIVE_Pin|Y_NEGATIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X_NEGATIVE_Pin Y_POSITIVE_Pin X_POSITIVE_Pin Y_NEGATIVE_Pin */
  GPIO_InitStruct.Pin = X_NEGATIVE_Pin|Y_POSITIVE_Pin|X_POSITIVE_Pin|Y_NEGATIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
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
