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

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SETA HAL_GPIO_WritePin(SEGA_GPIO_Port, SEGA_Pin, GPIO_PIN_RESET)
#define SETB HAL_GPIO_WritePin(SEGB_GPIO_Port, SEGB_Pin, GPIO_PIN_RESET)
#define SETC HAL_GPIO_WritePin(SEGC_GPIO_Port, SEGC_Pin, GPIO_PIN_RESET)
#define SETD HAL_GPIO_WritePin(SEGD_GPIO_Port, SEGD_Pin, GPIO_PIN_RESET)
#define SETE HAL_GPIO_WritePin(SEGE_GPIO_Port, SEGE_Pin, GPIO_PIN_RESET)
#define SETF HAL_GPIO_WritePin(SEGF_GPIO_Port, SEGF_Pin, GPIO_PIN_RESET)
#define SETG HAL_GPIO_WritePin(SEGG_GPIO_Port, SEGG_Pin, GPIO_PIN_RESET)
#define SETDOT HAL_GPIO_WritePin(SEGDOT_GPIO_Port, SEGDOT_Pin, GPIO_PIN_RESET)

#define RESETA HAL_GPIO_WritePin(SEGA_GPIO_Port, SEGA_Pin, GPIO_PIN_SET)
#define RESETB HAL_GPIO_WritePin(SEGB_GPIO_Port, SEGB_Pin, GPIO_PIN_SET)
#define RESETC HAL_GPIO_WritePin(SEGC_GPIO_Port, SEGC_Pin, GPIO_PIN_SET)
#define RESETD HAL_GPIO_WritePin(SEGD_GPIO_Port, SEGD_Pin, GPIO_PIN_SET)
#define RESETE HAL_GPIO_WritePin(SEGE_GPIO_Port, SEGE_Pin, GPIO_PIN_SET)
#define RESETF HAL_GPIO_WritePin(SEGF_GPIO_Port, SEGF_Pin, GPIO_PIN_SET)
#define RESETG HAL_GPIO_WritePin(SEGG_GPIO_Port, SEGG_Pin, GPIO_PIN_SET)
#define RESETDOT HAL_GPIO_WritePin(SEGDOT_GPIO_Port, SEGDOT_Pin, GPIO_PIN_SET)

#define DIG1ON HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_SET)
#define DIG2ON HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_SET)
#define DIG3ON HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, GPIO_PIN_SET)
#define DIG4ON HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, GPIO_PIN_SET)

#define DIG1OFF HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET)
#define DIG2OFF HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET)
#define DIG3OFF HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, GPIO_PIN_RESET)
#define DIG4OFF HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, GPIO_PIN_RESET)

#define INPUT 0
#define OUTPUT 1

uint16_t photoresistor, adc_display_flag;
int8_t dig1, dig2, dig3, dig4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);

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
	RESETDOT;
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
	// with DOT
	case 10:
		SETA;
		SETB;
		SETC;
		SETD;
		SETE;
		SETF;
		SETDOT;
		break;
	case 11:
		SETB;
		SETC;
		SETDOT;
		break;
	case 12:
		SETA;
		SETB;
		SETG;
		SETE;
		SETD;
		SETDOT;
		break;
	case 13:
		SETA;
		SETB;
		SETG;
		SETC;
		SETD;
		SETDOT;
		break;
	case 14:
		SETF;
		SETG;
		SETB;
		SETC;
		SETDOT;
		break;
	case 15:
		SETA;
		SETF;
		SETG;
		SETC;
		SETD;
		SETDOT;
		break;
	case 16:
		SETF;
		SETE;
		SETD;
		SETC;
		SETG;
		SETDOT;
		break;
	case 17:
		SETA;
		SETB;
		SETC;
		SETDOT;
		break;
	case 18:
		SETA;
		SETB;
		SETC;
		SETD;
		SETE;
		SETF;
		SETG;
		SETDOT;
		break;
	case 19:
		SETA;
		SETB;
		SETC;
		SETF;
		SETG;
		SETDOT;
		break;
	// middle segment SET with dot (20) and without (21)
	case 20:
		SETG;
		SETDOT;
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
		RESETDOT;
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
	dig3 = temp_dig3 + 10; //for digit with DOT
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

void seconds2dig(uint16_t seconds) {
	uint8_t temp4, temp2, minutes, sec;

	minutes = seconds / 60;
	sec = seconds - minutes * 60;

	if (minutes < 10) {
		dig4 = -1;
		temp4 = 0;
	} else {
		dig4 = minutes / 10;
		temp4 = dig4;
	}

	if (minutes < 1) {
		dig3 = -1;
	} else {
		dig3 = minutes - (temp4 * 10) + 10; //with DOT
	}

	if (seconds < 10) {
		dig2 = -1;
		temp2 = 0;
	} else {
		dig2 = sec / 10;
		temp2 = dig2;
	}

	dig1 = sec - temp2 * 10;

}

void change_state(GPIO_TypeDef* GPIO, uint16_t Pin, uint8_t state) {
	GPIO_InitTypeDef GPIO_InitStruct;
	switch(state) {
	case INPUT:
		GPIO_InitStruct.Pin = Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;
	case OUTPUT:
		GPIO_InitStruct.Pin = Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		break;
	}
	HAL_GPIO_Init(GPIO, &GPIO_InitStruct);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Switch_Pin) {
		adc_display_flag = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);

	if (htim->Instance == TIM10) {
		static uint16_t cnt1hz = 0;
		static uint8_t segment = 1;
		static uint16_t counter = 0;

		DIG1OFF;
		DIG2OFF;
		DIG3OFF;
		DIG4OFF;

		cnt1hz++;
		if (cnt1hz == 200) {
			cnt1hz = 0;
			counter++;
			if (counter == 6000) {
				counter = 0;
			}
		}

		if (adc_display_flag) {
			number2dig(photoresistor);
		} else {
			seconds2dig(counter);
		}

		switch (segment) {
		case 1:
			DIG1ON;
			bcd2seg(dig1);
//			change_state(PB7_GPIO_Port, PB7_Pin, OUTPUT);
//			change_state(PB8_GPIO_Port, PB8_Pin, INPUT);
//			change_state(PB9_GPIO_Port, PB9_Pin, INPUT);
			break;
		case 2:
			DIG2ON;
			bcd2seg(dig2);
//			change_state(PB8_GPIO_Port, PB8_Pin, OUTPUT);
//			change_state(PB7_GPIO_Port, PB7_Pin, INPUT);
//			change_state(PB9_GPIO_Port, PB9_Pin, INPUT);
			break;
		case 3:
			DIG3ON;
			bcd2seg(dig3);
//			change_state(PB9_GPIO_Port, PB9_Pin, OUTPUT);
//			change_state(PB7_GPIO_Port, PB7_Pin, INPUT);
//			change_state(PB8_GPIO_Port, PB8_Pin, INPUT);
			break;
		case 4:
			DIG4ON;
			bcd2seg(dig4);
			break;
		}

		segment++;
		if (segment > 4)
			segment = 1;
	}
}
//void HAL_SYSTICK_Callback(void) { //1kHz
//	static uint8_t cnt200hz = 0;
//	static uint16_t cnt1hz = 0;
//	static uint8_t segment = 1;
//	static uint16_t counter = 0;
//
//	cnt200hz++;
//	cnt1hz++;
//
//	if (cnt1hz == 1000) {
//		cnt1hz = 0;
//		counter++;
//		if (counter == 6000) {
//			counter = 0;
//		}
//	}
//
//	if (cnt200hz == 5) {
//		cnt200hz = 0;
//		DIG1OFF;
//		DIG2OFF;
//		DIG3OFF;
//		DIG4OFF;
//
//		if (adc_display_flag) {
//			number2dig(photoresistor);
//		} else {
//			seconds2dig(counter);
//		}
//
//		switch (segment) {
//		case 1:
//			DIG1ON;
//			bcd2seg(dig1);
////			change_state(PB7_GPIO_Port, PB7_Pin, OUTPUT);
////			change_state(PB8_GPIO_Port, PB8_Pin, INPUT);
////			change_state(PB9_GPIO_Port, PB9_Pin, INPUT);
//			break;
//		case 2:
//			DIG2ON;
//			bcd2seg(dig2);
////			change_state(PB8_GPIO_Port, PB8_Pin, OUTPUT);
////			change_state(PB7_GPIO_Port, PB7_Pin, INPUT);
////			change_state(PB9_GPIO_Port, PB9_Pin, INPUT);
//			break;
//		case 3:
//			DIG3ON;
//			bcd2seg(dig3);
////			change_state(PB9_GPIO_Port, PB9_Pin, OUTPUT);
////			change_state(PB7_GPIO_Port, PB7_Pin, INPUT);
////			change_state(PB8_GPIO_Port, PB8_Pin, INPUT);
//			break;
//		case 4:
//			DIG4ON;
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

/* USER CODE END PFP */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &photoresistor, 1);
	HAL_TIM_Base_Start_IT(&htim10);

//	HAL_GPIO_WritePin(PB6_GPIO_Port, PB6_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

//		change_state(PB7_GPIO_Port, PB7_Pin, OUTPUT);
//		change_state(PB8_GPIO_Port, PB8_Pin, INPUT);
//		change_state(PB9_GPIO_Port, PB9_Pin, INPUT);
//		HAL_Delay(1000);

//		change_state(PB8_GPIO_Port, PB8_Pin, OUTPUT);
//		change_state(PB7_GPIO_Port, PB7_Pin, INPUT);
//		change_state(PB9_GPIO_Port, PB9_Pin, INPUT);
//		HAL_Delay(1000);

//		change_state(PB9_GPIO_Port, PB9_Pin, OUTPUT);
//		change_state(PB7_GPIO_Port, PB7_Pin, INPUT);
//		change_state(PB8_GPIO_Port, PB8_Pin, INPUT);
//		HAL_Delay(1000);
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 10000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 50;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin 
                          |SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin 
                          |SEGE_Pin|SEGF_Pin|SEGG_Pin|SEGDOT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PB6_Pin|PB7_Pin|PB8_Pin|PB9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIG1_Pin DIG2_Pin DIG3_Pin DIG4_Pin 
                           SEGA_Pin SEGB_Pin SEGC_Pin SEGD_Pin 
                           SEGE_Pin SEGF_Pin SEGG_Pin SEGDOT_Pin */
  GPIO_InitStruct.Pin = DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin 
                          |SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin 
                          |SEGE_Pin|SEGF_Pin|SEGG_Pin|SEGDOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch_Pin */
  GPIO_InitStruct.Pin = Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6_Pin PB7_Pin PB8_Pin PB9_Pin */
  GPIO_InitStruct.Pin = PB6_Pin|PB7_Pin|PB8_Pin|PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
