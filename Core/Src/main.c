/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <ssd1306.h>
#include <ssd1306_fonts.h>
#include "horse_anim.h"
#include "png.h"
#include <stdio.h>
#include <string.h>
#include "auxiliary_material.h"
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	ssd1306_Init();
	MPU6050_Init();
	BMP280_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (!current_screen) {

			ssd1306_Fill(0);
			ssd1306_DrawBitmap(4, 2, bitmap_icon_dashboard, 16, 16, 1);
			ssd1306_SetCursor(25, 5);
			ssd1306_WriteString("Accelerometer", Font_7x10, 1);
			ssd1306_DrawBitmap(4, 24, bitmap_icon_parksensor, 16, 16, 1);
			ssd1306_SetCursor(25, 5 + 20 + 2);
			ssd1306_WriteString("Gyroscope", Font_7x10, 1);
			ssd1306_DrawBitmap(4, 46, bitmap_icon_fireworks, 16, 16, 1);
			ssd1306_SetCursor(25, 5 + 20 + 20 + 2 + 2);
			ssd1306_WriteString("Animation", Font_7x10, 1);

			if (cursor == 0) {
				ssd1306_DrawBitmap(0, 43, bitmap_item_sel_outline, 128, 21, 0);
				ssd1306_DrawBitmap(0, 22, bitmap_item_sel_outline, 128, 21, 0);
				ssd1306_DrawBitmap(0, 1, bitmap_item_sel_outline, 128, 21, 1);
			} else if (cursor == 1) {
				ssd1306_DrawBitmap(0, 43, bitmap_item_sel_outline, 128, 21, 0);
				ssd1306_DrawBitmap(0, 22, bitmap_item_sel_outline, 128, 21, 1);
				ssd1306_DrawBitmap(0, 1, bitmap_item_sel_outline, 128, 21, 0);
			} else {
				ssd1306_DrawBitmap(0, 43, bitmap_item_sel_outline, 128, 21, 1);
				ssd1306_DrawBitmap(0, 22, bitmap_item_sel_outline, 128, 21, 0);
				ssd1306_DrawBitmap(0, 1, bitmap_item_sel_outline, 128, 21, 0);
			}

			if (HAL_GPIO_ReadPin(GPIOE, UP_BUTTON)) {
				cursor--;
				if (cursor == -1)
					cursor = 2;
				HAL_Delay(100);

			}

			if (HAL_GPIO_ReadPin(GPIOE, DOWN_BUTTON)) {
				cursor++;
				if (cursor == 3)
					cursor = 0;
				HAL_Delay(100);
			}

			if (HAL_GPIO_ReadPin(GPIOE, ENTER_BUTTON)) {
				current_screen = !current_screen;
				HAL_Delay(200);
			}
		}

		if (current_screen) {

			ssd1306_Fill(0);

			if (cursor == 0) {
				MPU6500_Read_Values();
				char buffer_float[7];
				ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
				ssd1306_SetCursor(5, 16); //Posiciona o "cursor" no pixel correspondente
				ssd1306_WriteString("Accel x: ", Font_6x8, 1); //Escreve o texto no buffer
				sprintf(buffer_float, "%.1f", Ax);
				ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
				ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
				ssd1306_WriteString("Accel y: ", Font_6x8, 1); //Escreve o texto no buffer
				sprintf(buffer_float, "%.1f", Ay);
				ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
				ssd1306_SetCursor(5, 44); //Posiciona o "cursor" no pixel correspondente
				ssd1306_WriteString("Accel z: ", Font_6x8, 1); //Escreve o texto no buffer
				sprintf(buffer_float, "%.1f", Az);
				ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
			}

			else if (cursor == 1) {
				MPU6500_Read_Values();
				char buffer_floats[7];
				ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
				ssd1306_SetCursor(5, 16); //Posiciona o "cursor" no pixel correspondente
				ssd1306_WriteString("Gyro x: ", Font_6x8, 1); //Escreve o texto no buffer
				sprintf(buffer_floats, "%.1f", gx);
				ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no buffer
				ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
				ssd1306_WriteString("Gyro y: ", Font_6x8, 1); //Escreve o texto no buffer
				sprintf(buffer_floats, "%.1f", gy);
				ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no buffer
				ssd1306_SetCursor(5, 44); //Posiciona o "cursor" no pixel correspondente
				ssd1306_WriteString("Gyro z: ", Font_6x8, 1); //Escreve o texto no buffer
				sprintf(buffer_floats, "%.1f", gz);
				ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no bufferr

			} else
				animation();

			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)) {
				current_screen = !current_screen;
				HAL_Delay(200);
			}
		}

		ssd1306_DrawBitmap(128-8, 0, bitmap_scrollbar_background, 8, 64, 1);
		ssd1306_UpdateScreen();

//		BMP280_Read_Measures(&t, &p);
//		printf("Temperature: %.1f\n", t);
//		printf("Pressure: %.1f\n\n", p);
//		HAL_Delay(200);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
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
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 1000000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : PE9 PE10 PE11 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
	USART1->DR = (ch & (uint16_t) 0x01FF);
	while (!(USART1->SR & USART_SR_TXE))
		; //espera pelo fim da transmissão do caractere para evitar a segunda transmissão antes da primeira ser concluída
	return ch;
}
int __io_getchar(void) {
	return (uint16_t) (USART1->DR & (uint16_t) 0x01FF);
}
//ISR da USART1. Todas as ISR's estão definidas no arquivo startup_stm32.s
void USART1_IRQHandler(void) {
	__io_putchar(__io_getchar());
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
