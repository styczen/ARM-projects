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
#include "defines.h"
#include "matrix.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

float current_angle_comp;
float x_post[2], P_post[4], K[2];
float dt_mean, dt_sum, dt;

//float u[1][1] = { { 0 } };
//float x_post[2][1] = { { 0 }, { 0 } };
//float Ax[2][1] = { { 0 }, { 0 } };
//float Bu[2][1] = { { 0 }, { 0 } };
//float x_pri[2][1] = { { 0 }, { 0 } };
//float P_post[2][2] = { { 0, 0 }, { 0, 0 } };
//float AP[2][2] = { { 0, 0 }, { 0, 0 } };
//float AT[2][2] = { { 0, 0 }, { 0, 0 } };
//float APAT[2][2] = { { 0, 0 }, { 0, 0 } };
//float V[2][2] = { { 0, 0 }, { 0, 0 } };
//float P_pri[2][2] = { { 0, 0 }, { 0, 0 } };
//float y[1][1] = { { 0 } };
//float Cx[1][1] = { { 0 } };
//float eps[1][1] = { { 0 } };
//float CP[1][2] = {{0, 0}};
//float CPCT[1][1] = { { 0 } };
//float S[1][1] = { { 0 } };
//float W[1][1] = { { 0 } };
//float PCT[2][1] = { { 0 }, { 0 } };
//float S1[1][1] = { { 0 } };
//float K[2][1] = { { 0 }, { 0 } };
//float Keps[2][1] = { { 0 }, { 0 } };
//float KS[2][1] = { { 0 }, { 0 } };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void read_accel_g(float* tab);
void read_gyro_omega(float* tab);
void read_gyro_raw(uint8_t* tab);
void read_accel_raw(uint8_t* tab);
uint8_t read_gyro_byte(uint8_t address);
void send_settings(uint8_t address, uint8_t settings);

void accelerometer_config(void);
void gyroscope_config(void);
void magnetometer_config(void); // TODO
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	float u[1], Ax[2], Bu[2], x_pri[2], AP[4], AT[4], APAT[4], V[4], P_pri[4], y[1],
	Cx[1], eps[1], CP[2], CPCT[1], S[1], W[1], PCT[2], S1[1], Keps[2], KS[2], KSKT[4],
	A[4], B[2], C[2], std_dev_v, std_dev_w;

	float accel_axis_g[3], gyro_axis_omega[3], accel_angle, prev_angle_comp=0;

	uint16_t i = 0;

	uint32_t currTimeStamp = 0;
	static uint32_t prevTimeStamp = 0;

	//********************************************************
	//Change this variables:
	std_dev_v = 1.0;
	std_dev_w = 2.0;
	/* Wartosci poczatkowe filtru */
	P_post[0] = 1.0;
	P_post[1] = 0.0;
	P_post[2] = 0.0;
	P_post[3] = 1.0;
	//********************************************************

	//**************Variables initialization******************
	dt = 0.000304; //seconds
	A[0] = 1;
	A[1] = -dt;
	A[2] = 0;
	A[3] = 1;

	B[0] = dt;
	B[1] = 0;

	C[0] = 1;
	C[1] = 0;

//	V[0] = std_dev_v * std_dev_v * dt;
//	V[1] = 0;
//	V[2] = 0;
//	V[3] = std_dev_v * std_dev_v * dt;

	V[0] = 0;
	V[1] = 0;
	V[2] = 0;
	V[3] = 0;

	W[0] = std_dev_w * std_dev_w;
	//********************************************************


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

	/* USER CODE BEGIN 2 */
	accelerometer_config();
	gyroscope_config();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		currTimeStamp = HAL_GetTick();
		dt = (float) (currTimeStamp - prevTimeStamp) / 1000.0; //seconds
		prevTimeStamp = currTimeStamp;

		A[1] = -dt;
		B[0] = dt;
		read_accel_g(accel_axis_g);
		read_gyro_omega(gyro_axis_omega);

		accel_angle = atan2(accel_axis_g[0], accel_axis_g[2]) * RAD_TO_DEG;
		//Complementary filter
		current_angle_comp = 0.80
				* (prev_angle_comp + (gyro_axis_omega[1] * dt))
				+ 0.20 * accel_angle;
		prev_angle_comp = current_angle_comp;

		//Kalman filter
		//Predykcja
		/* x(t+1|t) = Ax(t|t) + Bu(t) */
		u[0] = gyro_axis_omega[1];
		matrix_2x2_mul_2x1(A, x_post, Ax);
		matrix_2x1_mul_1x1(B, u, Bu);
		matrix_2x1_add_2x1(Ax, Bu, x_pri);

		/* P(t+1|t) = AP(t|t)A^T + V */
		matrix_2x2_mul_2x2(A, P_post, AP);
		matrix_2x2_trans(A, AT);
		matrix_2x2_mul_2x2(AP, AT, APAT);
		matrix_2x2_add_2x2(APAT, V, P_pri);

		//Korekcja
		/* eps(t) = y(t) - Cx(t|t-1) */
		y[0] = accel_angle;
		matrix_1x2_mul_2x1(C, x_pri, Cx);
		eps[0] = y[0] - Cx[0];

		/* S(t) = CP(t|t-1)C^T + W */
		matrix_1x2_mul_2x2(C, P_pri, CP);
		matrix_1x2_mul_2x1(C, C, CPCT);
		S[0] = CPCT[0] + W[0];

		/* K(t) = P(t|t-1)C^TS(t)^-1 */
		matrix_2x2_mul_2x1(P_pri, C, PCT);
		S1[0] = 1.0 / S[0];
		matrix_2x1_mul_1x1(PCT, S1, K);

		/* x(t|t) = x(t|t-1) + K(t)eps(t) */
		matrix_2x1_mul_1x1(K, eps, Keps);
		matrix_2x1_add_2x1(x_pri, Keps, x_post);

		/* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
		matrix_2x1_mul_1x1(K, S, KS);
		matrix_2x1_mul_1x2(KS, K, KSKT);
		matrix_2x2_sub_2x2(P_pri, KSKT, P_post);

		dt_sum += dt;
		i++;
		if (i == 10000) {
			i = 0;
			dt_mean = dt_sum / 10000;
			dt_sum = 0;
		}
//		HAL_Delay(100);
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
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

/* SPI1 init function */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
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
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void send_settings(uint8_t address, uint8_t settings) {
	CS_LOW;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Transmit(&hspi1, &settings, 1, 50);
	CS_HIGH;
}

void accelerometer_config(void) {
	uint8_t settings = 0b01010111; //100Hz, XYZ axis enable
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

void gyroscope_config(void) {
	send_settings(L3GD20_REGISTER_CTRL_REG1, 0b11001111); //output data rate, XYZ axis enable
	send_settings(L3GD20_REGISTER_CTRL_REG2, 0b00000000); //high pass filter normal mode and cutoff freq
	send_settings(L3GD20_REGISTER_CTRL_REG3, 0b00000000); //nothing
	send_settings(L3GD20_REGISTER_CTRL_REG4, 0b00000000); //continous update, 250 dps
	send_settings(L3GD20_REGISTER_CTRL_REG5, 0b00010000); //high pass filter enable
}

void magnetometer_config(void) {

}

uint8_t read_gyro_byte(uint8_t address) {
	CS_LOW;
	address |= 0x80;
	uint8_t data;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &data, 1, 50);
	CS_HIGH;
	return data;
}

void read_gyro_raw(uint8_t* tab) {
	uint8_t i, adr;
//	tab[0] = read_gyro_byte(L3GD20_REGISTER_OUT_X_L);
//	tab[1] = read_gyro_byte(L3GD20_REGISTER_OUT_X_H);
//	tab[2] = read_gyro_byte(L3GD20_REGISTER_OUT_Y_L);
//	tab[3] = read_gyro_byte(L3GD20_REGISTER_OUT_Y_H);
//	tab[4] = read_gyro_byte(L3GD20_REGISTER_OUT_Z_L);
//	tab[5] = read_gyro_byte(L3GD20_REGISTER_OUT_Z_H);
	for (i = 0, adr = L3GD20_REGISTER_OUT_X_L; i < 6; i++, adr++)
		tab[i] = read_gyro_byte(adr);
}

void read_accel_raw(uint8_t* tab) {
	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_OUT_XYZ_MULTIREAD_A,
			1, tab, 6, 50);
}

void read_accel_g(float* tab) {
	uint8_t raw[6], i, j;
	read_accel_raw(raw);
	int16_t accel_raw[3];
	for (i = 0, j = 0; i < 3; i++, j += 2) {
		accel_raw[i] = ((raw[j + 1] << 8) | raw[j]);
		tab[i] =
				(float) ((accel_raw[i] * LSM303_ACC_SCALE) / (float) INT16_MAX);
	}
}
void read_gyro_omega(float* tab) {
	uint8_t raw[6], i, j;
	read_gyro_raw(raw);
	int16_t gyro_raw[3];
	for (i = 0, j = 0; i < 3; i++, j += 2) {
		gyro_raw[i] = ((raw[j + 1] << 8) | raw[j]);
		tab[i] = gyro_raw[i] * L3GD20_SENSITIVITY_250;
	}
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
