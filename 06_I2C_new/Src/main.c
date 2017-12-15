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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define LSM303_ACC_ADDRESS 0b00110011 //(0x19 << 1) // adres akcelerometru: 0011 001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1
#define LSM303_ACC_CTRL_REG3_A 0x22 // rejestr ustawien 3
#define LSM303_ACC_CTRL_REG4_A 0x23 // rejestr ustawien 4
#define LSM303_ACC_Z_H_A 0b00101101 //0x2D // wyzszy bajt danych osi Z
#define LSM303_ACC_Z_L_A 0x2C // nizszy bajt danych osi Z
#define LSM303_ACC_X_L_A 0x28 // nizszy bajt danych osi X

// mlodszy bajt danych osi Z z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_Z_L_A_MULTI_READ (LSM303_ACC_Z_L_A | 0x80)

// mlodszy bajt danych osi X z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_X_L_A_MULTI_READ (LSM303_ACC_X_L_A | 0x80)

// Maski bitowe
// CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPEN][ZEN][YEN][XEN]
#define LSM303_ACC_Z_ENABLE 0x04 // 0000 0100
#define LSM303_ACC_XYZ_ENABLE 0x07 // 0000 0111
#define LSM303_ACC_100HZ 0x50 //0101 0000
#define LSM303_ACC_1HZ 0x10 //0001 0000

// CTRL_REG3_A = [CLICK][AOI1][AOI2][DRDY_1][DRDY_2][WTM][OVERRUN][---]
#define LSM303_ACC_I1_DRDY1 0x10 //0001 0000
#define LSM303_ACC_I1_DRDY2 0x08 //0000 1000

//CTRL_REG4_A = [BDU][BLE][FS1][FS0][HR][0][0][SIM]
#define LSM303_ACC_16G 0x30 // zakres 16 g

#define LSM303_ACC_RESOLUTION 2.0 // Maksymalna wartosc mierzalnego przyspieszenia [g]

// Zmienne
uint8_t Data[6]; // Zmienna do bezposredniego odczytu danych z akcelerometru
int16_t Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi X
int16_t Yaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Y
int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Z

float Xaxis_g = 0; // Zawiera przyspieszenie w osi X przekstalcone na jednostke fizyczna [g]
float Yaxis_g = 0; // Zawiera przyspieszenie w osi Y przekstalcone na jednostke fizyczna [g]
float Zaxis_g = 0; // Zawiera przyspieszenie w osi Z przekstalcone na jednostke fizyczna [g]

//float R_g = 0; // Dlugosc wektora przyspieszenia bedacego zlozeniem przyspieszenia kazdej z osi [g]
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
	// wypelnieine zmiennej konfiguracyjnej odpowiednimi opcjami
	uint8_t Settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;

	// Wpisanie konfiguracji do rejestru akcelerometru
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);


//	Settings = LSM303_ACC_16G;
//	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG4_A, 1, &Settings, 1, 100);

//	Settings = LSM303_ACC_I1_DRDY2;
//	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG3_A, 1, &Settings, 1, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// Pobranie 6 bajtow danych zawierajacych przyspieszenia w 3 osiach
		HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS,
				LSM303_ACC_X_L_A_MULTI_READ, 1, Data, 6, 100);

		// Konwersja odebranych bajtow danych na typ int16_t
		Xaxis = ((Data[1] << 8) | Data[0]);
		Yaxis = ((Data[3] << 8) | Data[2]);
		Zaxis = ((Data[5] << 8) | Data[4]);

		// obliczenie przyspieszen w kazdej z osi w jednostce SI [g]
		Xaxis_g = ((float) Xaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
		Yaxis_g = ((float) Yaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
		Zaxis_g = ((float) Zaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;

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

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
