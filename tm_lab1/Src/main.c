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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SETA HAL_GPIO_WritePin(SEGA_GPIO_Port, SEGA_Pin, GPIO_PIN_SET)
#define SETB HAL_GPIO_WritePin(SEGB_GPIO_Port, SEGB_Pin, GPIO_PIN_SET)
#define SETC HAL_GPIO_WritePin(SEGC_GPIO_Port, SEGC_Pin, GPIO_PIN_SET)
#define SETD HAL_GPIO_WritePin(SEGD_GPIO_Port, SEGD_Pin, GPIO_PIN_SET)
#define SETE HAL_GPIO_WritePin(SEGE_GPIO_Port, SEGE_Pin, GPIO_PIN_SET)
#define SETF HAL_GPIO_WritePin(SEGF_GPIO_Port, SEGF_Pin, GPIO_PIN_SET)
#define SETG HAL_GPIO_WritePin(SEGG_GPIO_Port, SEGG_Pin, GPIO_PIN_SET)
#define SETDP HAL_GPIO_WritePin(SEGDP_GPIO_Port, SEGDP_Pin, GPIO_PIN_SET)

#define RESETA HAL_GPIO_WritePin(SEGA_GPIO_Port, SEGA_Pin, GPIO_PIN_RESET)
#define RESETB HAL_GPIO_WritePin(SEGB_GPIO_Port, SEGB_Pin, GPIO_PIN_RESET)
#define RESETC HAL_GPIO_WritePin(SEGC_GPIO_Port, SEGC_Pin, GPIO_PIN_RESET)
#define RESETD HAL_GPIO_WritePin(SEGD_GPIO_Port, SEGD_Pin, GPIO_PIN_RESET)
#define RESETE HAL_GPIO_WritePin(SEGE_GPIO_Port, SEGE_Pin, GPIO_PIN_RESET)
#define RESETF HAL_GPIO_WritePin(SEGF_GPIO_Port, SEGF_Pin, GPIO_PIN_RESET)
#define RESETG HAL_GPIO_WritePin(SEGG_GPIO_Port, SEGG_Pin, GPIO_PIN_RESET)
#define RESETDP HAL_GPIO_WritePin(SEGDP_GPIO_Port, SEGDP_Pin, GPIO_PIN_RESET)

#define COM1ON HAL_GPIO_WritePin(COM1_GPIO_Port, COM1_Pin, GPIO_PIN_RESET)
#define COM2ON HAL_GPIO_WritePin(COM2_GPIO_Port, COM2_Pin, GPIO_PIN_RESET)
#define COM3ON HAL_GPIO_WritePin(COM3_GPIO_Port, COM3_Pin, GPIO_PIN_RESET)
#define COM4ON HAL_GPIO_WritePin(COM4_GPIO_Port, COM4_Pin, GPIO_PIN_RESET)

#define COM1OFF HAL_GPIO_WritePin(COM1_GPIO_Port, COM1_Pin, GPIO_PIN_SET)
#define COM2OFF HAL_GPIO_WritePin(COM2_GPIO_Port, COM2_Pin, GPIO_PIN_SET)
#define COM3OFF HAL_GPIO_WritePin(COM3_GPIO_Port, COM3_Pin, GPIO_PIN_SET)
#define COM4OFF HAL_GPIO_WritePin(COM4_GPIO_Port, COM4_Pin, GPIO_PIN_SET)

int8_t dig1, dig2, dig3, dig4;
uint16_t photo;
uint8_t flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void bcd2seg(int8_t digit) {
	RESETA;
	RESETB;
	RESETC;
	RESETD;
	RESETE;
	RESETF;
	RESETG;
	RESETDP;
	switch (digit) {
	case 0:
		SETA;
		SETB;
		SETC;
		SETD;
		SETE;
		SETF;
		break;
	case 1:
		SETB;
		SETC;
		break;
	case 2:
		SETA;
		SETB;
		SETG;
		SETE;
		SETD;
		break;
	case 3:
		SETA;
		SETB;
		SETG;
		SETC;
		SETD;
		break;
	case 4:
		SETF;
		SETG;
		SETB;
		SETC;
		break;
	case 5:
		SETA;
		SETF;
		SETG;
		SETC;
		SETD;
		break;
	case 6:
		SETF;
		SETE;
		SETD;
		SETC;
		SETG;
		break;
	case 7:
		SETA;
		SETB;
		SETC;
		break;
	case 8:
		SETA;
		SETB;
		SETC;
		SETD;
		SETE;
		SETF;
		SETG;
		break;
	case 9:
		SETA;
		SETB;
		SETC;
		SETF;
		SETG;
		break;
		// with DP
	case 10:
		SETA;
		SETB;
		SETC;
		SETD;
		SETE;
		SETF;
		SETDP;
		break;
	case 11:
		SETB;
		SETC;
		SETDP;
		break;
	case 12:
		SETA;
		SETB;
		SETG;
		SETE;
		SETD;
		SETDP;
		break;
	case 13:
		SETA;
		SETB;
		SETG;
		SETC;
		SETD;
		SETDP;
		break;
	case 14:
		SETF;
		SETG;
		SETB;
		SETC;
		SETDP;
		break;
	case 15:
		SETA;
		SETF;
		SETG;
		SETC;
		SETD;
		SETDP;
		break;
	case 16:
		SETF;
		SETE;
		SETD;
		SETC;
		SETG;
		SETDP;
		break;
	case 17:
		SETA;
		SETB;
		SETC;
		SETDP;
		break;
	case 18:
		SETA;
		SETB;
		SETC;
		SETD;
		SETE;
		SETF;
		SETG;
		SETDP;
		break;
	case 19:
		SETA;
		SETB;
		SETC;
		SETF;
		SETG;
		SETDP;
		break;
		// middle segment SET with DP (20) and without (21)
	case 20:
		SETG;
		SETDP;
		break;
	case 21:
		SETG;
		break;
	default:
		RESETA;
		RESETB;
		RESETC;
		RESETD;
		RESETE;
		RESETF;
		RESETG;
		RESETDP;
		break;
	}
}

void number2dig(uint16_t number) {
	uint8_t temp_dig2, temp_dig3, temp_dig4;

	temp_dig4 = (number / 1000);
	dig4 = temp_dig4;
	if (!temp_dig4) {
		if (number < 1000)
			dig4 = -1;
	}

	temp_dig3 = ((number - temp_dig4 * 1000) / 100);
	dig3 = temp_dig3; //for digit with DP
	if (!temp_dig3)
		if (number < 100)
			dig3 = -1;

	temp_dig2 = ((number - temp_dig4 * 1000 - temp_dig3 * 100) / 10);
	dig2 = temp_dig2;
	if (!temp_dig2)
		if (number < 10)
			dig2 = -1;

	dig1 = (number - temp_dig4 * 1000 - temp_dig3 * 100 - temp_dig2 * 10);
}

//void HAL_SYSTICK_Callback(void) { //1kHz
//	static uint8_t segment = 1;
//	static uint16_t cnt1hz = 0;
//	static uint8_t cnt200hz = 0;
//	static uint16_t number = 0;
//
//	cnt1hz++;
//	cnt200hz++;
//	if (cnt1hz == 100) {
//		cnt1hz = 0;
//		number++;
//		if (number == 10000) {
//			number = 0;
//		}
//	}
//
//	number2dig(number);
//	if (cnt200hz == 5) {
//		cnt200hz = 0;
//		COM1OFF;
//		COM2OFF;
//		COM3OFF;
//		COM4OFF;
//
//		switch (segment) {
//		case 1:
//			COM1ON;
//			bcd2seg(dig1);
//			break;
//		case 2:
//			COM2ON;
//			bcd2seg(dig2);
//			break;
//		case 3:
//			COM3ON;
//			bcd2seg(dig3);
//			break;
//		case 4:
//			COM4ON;
//			bcd2seg(dig4);
//			break;
//		}
//
//		segment++;
//		if (segment > 4)
//			segment = 1;
//
//	}
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM11) {
		static uint8_t segment = 1;
		static uint16_t cnt1hz = 0;
		static uint16_t number = 0;
		COM1OFF;
		COM2OFF;
		COM3OFF;
		COM4OFF;

		cnt1hz++;
		if (cnt1hz == 200) {
			cnt1hz = 0;
			number++;
			if (number == 10000) {
				number = 0;
			}
		}

		if (flag) {
			number2dig(photo);
		} else {
			number2dig(number);
		}

		switch (segment) {
		case 1:
			COM1ON;
			bcd2seg(dig1);
			break;
		case 2:
			COM2ON;
			bcd2seg(dig2);
			break;
		case 3:
			COM3ON;
			bcd2seg(dig3);
			break;
		case 4:
			COM4ON;
			bcd2seg(dig4);
			break;
		}

		segment++;
		if (segment > 4)
			segment = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		photo = HAL_ADC_GetValue(&hadc1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case Switch1_Pin:
		flag = 1;
		break;
	case Switch2_Pin:
		flag = 0;
		break;
	default:
		break;
	}
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t data[20], size;
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
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim11);
	HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		size = sprintf((char*)data, "%d\n\r", (int)&photo);
		HAL_UART_Transmit_DMA(&huart1, data, size);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/* USER CODE BEGIN 4 */

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
