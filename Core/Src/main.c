/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <stdbyte.h>
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

/* USER CODE BEGIN PV */
////for_UART
//for_recive
uint8_t rx_data[57];
uint8_t rx_length;

//for_transmit
uint8_t tx_data[] = { 0x02, 0x01, 0x31, 0xCD };
uint8_t tx_data_pusk[] = { 0x02, 0x03, 0x36, 0x00, 0x00, 0xC8 };
uint8_t tx_data_ostanov[] = { 0x02, 0x01, 0x37, 0xCB };

////for_SPI
uint8_t answer[] = { 0x00, 0x00 };
uint8_t ask[] = { 0x00, 0x00 };

////for_command
uint8_t command = 0;
uint8_t command_ask = 0;

//flags
uint8_t flagT = 0;
uint8_t flagR = 0;
uint8_t flagSPI = 0;
uint8_t flagTemp = 0;

//timeStop
uint32_t timeStopR;

//Temperature
uint8_t Tr[] = { 0x00, 0x00, 0x00, 0x00 };
uint8_t Tz[] = { 0x00, 0x00, 0x00, 0x00 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (answer[0] != 0x00) {
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, RESET);
		}



		//		Check_Temp();
				//begin_check_temp
				float tr = *(float*) &Tr;
				float tz = *(float*) &Tz;

				if (answer[0] == 0x02) {
					if (tr >= tz && command_ask && !flagTemp) {
						//ohlajdaet
						command = 1;
						answer[1] = 0x00;
						command_ask = 0;
						flagTemp = 1;
					}
					if (tr < tz && flagTemp && !command_ask) {
						//ohladil
						command = 2;
						answer[1] = 0x01;
						flagTemp = 0;
					}
				}
					//end_check_temp




		//begin_transmit_receive

//		UART_Transmit();
		//begin_transmit
		switch (command) {
		case 1:
			HAL_UART_Transmit_IT(&huart1, tx_data_pusk, 6);
			rx_length = 1;
			break;
		case 2:
			HAL_UART_Transmit_IT(&huart1, tx_data_ostanov, 4);
			rx_length = 1;
			break;
		default:
			HAL_UART_Transmit_IT(&huart1, tx_data, 4);
			rx_length = 57;
			break;
		}
		while (!flagT) {
		}
		flagT = 0;
		//end_transmit


//		UART_Receive();
		//begin_receive
		timeStopR = HAL_GetTick();
		HAL_UART_Receive_IT(&huart1, rx_data, rx_length);
		while (!flagR) {
			if ((HAL_GetTick() - timeStopR) > 15) {
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, SET);
				flagR = 1;
				HAL_UART_AbortReceive(&huart1);
				answer[0] = 0x00;
			}
		}
		flagR = 0;
		//end_receive

		//end_transmit_receive

		//begin_SPI
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_IT(&hspi1, answer, ask, 2);
		while (!flagSPI) {
		}
		flagSPI = 0;
		//end_SPI

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {
		flagT = 1;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {
		switch (rx_length) {
		case 1:
			if (rx_data[0] == 0x06) {
				command = 0;
			}
			break;
		default:
			switch (rx_data[23]) {
			case 0x01:
				answer[0] = 0x01;
				break;
			case 0x02:
				answer[0] = 0x02;
				break;
			case 0x00:
				answer[0] = 0x03;
				break;
			}

			//сохраняем температуру
			for (int var = 0; var < 3; ++var) {
				Tr[var] = rx_data[9 + var];
				Tz[var] = rx_data[26 + var];
			}

			//проверка на нагрев/охлаждение
			if (rx_data[44] == 0x01){
				flagTemp = 1;
			} else {
				flagTemp = 0;
			}

			break;
		}
		flagR = 1;
	}

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (hspi == &hspi1) {
		if (!hspi1.RxXferCount) {
			HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

			switch (ask[0]) {
			case 0x01:
				command_ask = 1;
				break;
			default:
				command_ask = 0;
				break;
			}

			flagSPI = 1;
		}
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
