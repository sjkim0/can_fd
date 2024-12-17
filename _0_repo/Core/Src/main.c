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
#include "fdcan.h"
#include "memorymap.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DEF_TX_DATA_LENGTH (12U)
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[DEF_TX_DATA_LENGTH] = {0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t RxData_0[12];
uint8_t RxData_1[12];
uint8_t RxData_FIFO_0[12];
uint8_t new_buff_flag = 0;
uint8_t new_fifo_0_flag = 0;
uint8_t new_fifo_1_flag = 0;

uint8_t rx_data_ok = 0;
uint8_t rx_remote_ok = 0;
uint8_t rx_fifo_range_ok = 0;
uint8_t rx_fifo_mask_ok = 0;

void filterSet(void);
void txBufferSet(void);
void notificationSet(void);
void txbufferStart(void);
void newMessageCheckLoop(void);
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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  filterSet();
  txBufferSet();

  HAL_FDCAN_Start(&hfdcan1);

  notificationSet();
  txbufferStart();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  newMessageCheckLoop();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance == FDCAN1)
	{
		if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
		{
			new_fifo_0_flag = 1;
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if(hfdcan->Instance == FDCAN1)
	{
		if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
		{
			new_fifo_1_flag = 1;
		}
	}
}

void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
	if(hfdcan->Instance == FDCAN1)
	{
		new_buff_flag = 1;
	}
}

void filterSet(void)
{
	/* Configure standard ID reception filter to Rx buffer 0 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
	sFilterConfig.FilterID1 = 0x100;
	sFilterConfig.FilterID2 = 0; // This parameter is ignored if FilterConfig is set to FDCAN_FILTER_TO_RXBUFFER.
	sFilterConfig.RxBufferIndex = FDCAN_RX_BUFFER0;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	/* Configure standard ID reception filter to Rx buffer 1 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
	sFilterConfig.FilterID1 = 0x101;
	sFilterConfig.FilterID2 = 0; // This parameter is ignored if FilterConfig is set to FDCAN_FILTER_TO_RXBUFFER.
	sFilterConfig.RxBufferIndex = FDCAN_RX_BUFFER1;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	/* Configure standard ID reception filter to Rx buffer 1 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 2;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x102;
	sFilterConfig.FilterID2 = 0x105; // This parameter is ignored if FilterConfig is set to FDCAN_FILTER_TO_RXBUFFER.
	// ignored buffer index in fifo mode
	// sFilterConfig.RxBufferIndex = FDCAN_RX_BUFFER2;

	/* Configure standard ID reception filter to Rx buffer 2 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 3;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x106;
	sFilterConfig.FilterID2 = 0x10F; // This parameter is ignored if FilterConfig is set to FDCAN_FILTER_TO_RXBUFFER.

	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	/* Configure global filter */
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
								FDCAN_ACCEPT_IN_RX_FIFO0,
								FDCAN_REJECT,
								FDCAN_FILTER_REMOTE,
								FDCAN_REJECT_REMOTE);
}

void txBufferSet(void)
{
	/* Configure Tx buffer message */
	TxHeader.Identifier = 0x100;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0x00; // Ignore because FDCAN_NO_TX_EVENTS
	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader, TxData, FDCAN_TX_BUFFER0);

	/* Configure Tx buffer message */
	TxHeader.Identifier = 0x101;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0x00; // Ignore because FDCAN_NO_TX_EVENTS
	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader, TxData, FDCAN_TX_BUFFER1);

	for(int i = 0; i < DEF_TX_DATA_LENGTH; i++)
	{
		TxData[i] = i;
	}
	/* Configure Tx buffer message */
	TxHeader.Identifier = 0x103;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0x00; // Ignore because FDCAN_NO_TX_EVENTS
	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader, TxData, FDCAN_TX_BUFFER2);

	for(int i = 0; i < DEF_TX_DATA_LENGTH; i++)
	{
		TxData[i] = i * 2;
	}
	/* Configure Tx buffer message */
	TxHeader.Identifier = 0x106;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0x00; // Ignore because FDCAN_NO_TX_EVENTS
	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader, TxData, FDCAN_TX_BUFFER3);
}

void notificationSet(void)
{
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 1);

	// ignored buffer index in fifo mode
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

void txbufferStart(void)
{
	HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER0);
	while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER0));

	HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER1);
	while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER1));

	HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER2);
	while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER2));

	HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER3);
	while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER2));
}

void newMessageCheckLoop(void)
{
	if(new_buff_flag == 1)
	{
		if(HAL_FDCAN_IsRxBufferMessageAvailable(&hfdcan1, FDCAN_RX_BUFFER0) == 1)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_BUFFER0, &RxHeader, RxData_0);
			if(RxHeader.RxFrameType == FDCAN_DATA_FRAME)
			{
				// data frame checker
				rx_data_ok = 1;
			}
		}
		if(HAL_FDCAN_IsRxBufferMessageAvailable(&hfdcan1, FDCAN_RX_BUFFER1) == 1)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_BUFFER1, &RxHeader, RxData_1);
			if(RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
			{
				// remote frame checker
				rx_remote_ok = 1;
			}
		}
		new_buff_flag = 0;
	}
	if(new_fifo_0_flag == 1)
	{
		// cannot use fdcan_isrxbuffermessageavailable
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData_FIFO_0);

		rx_fifo_range_ok = 1;
		new_fifo_0_flag = 0;
	}
	if(new_fifo_1_flag == 1)
	{
		rx_fifo_mask_ok = 1;
		new_fifo_1_flag = 0;
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
  while (1)
  {
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
