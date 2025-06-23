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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// they're in the main.h file, better methinks
//#include <string.h>
//#include <stdio.h>
//#include <math.h>
//#include "CAN_Transmit.h"
//#include "CAN_Receive.h"
//#include "gearselecttest.h"
//#include "led.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t dma_interrupts_enabled = 0; // Flag to control interrupt enabling
#define ADC_BUF_LEN 16
uint16_t adc_buf[ADC_BUF_LEN];

/////////////////////////////////////////////
//CAN variables
osThreadId parkedHandle;
osThreadId canTxTaskHandle;
osThreadId canRxTaskHandle;
osThreadId buttons_100ms_TaskHandle;
osThreadId buttons_500ms_TaskHandle;




QueueHandle_t CANq;

volatile uint8_t datacheck = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
void parked_init(void const *argument);
void can_tx(void const *argument);
void can_rx(void const *argument);

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
	CANq = xQueueCreate(100, sizeof(struct CANFrame));

	osThreadDef(parked, parked_init, osPriorityLow, 0, 32);
	parkedHandle = osThreadCreate(osThread(parked), NULL);

	osThreadDef(canTxTask, can_tx, osPriorityNormal, 0, 128);
	canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);

	osThreadDef(canRxTask, can_rx, osPriorityHigh, 0, 128);
	canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

	osThreadDef(buttons_task_100ms, buttons_100ms, osPriorityHigh, 0, 64);
	buttons_100ms_TaskHandle = osThreadCreate(osThread(buttons_task_100ms), NULL);

	osThreadDef(buttons_task_500ms, buttons_500ms, osPriorityHigh, 0, 128);
		buttons_500ms_TaskHandle = osThreadCreate(osThread(buttons_task_500ms), NULL);

//	osThreadDef(canRxTask, can_rx, osPriorityNormal, 0, 128);
//		canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);
//
//		osThreadId BrakeTaskHandle;
//		osThreadId LeftSignalTaskHandle;
//		osThreadId RightSignalTaskHandle;
//		osThreadId HazardTaskHandle;
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_CAN_Init();

	/* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of parked */

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();
	Enable_DMA_Interrupts();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
	PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };
	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

//  /** Configure Regular Channel
//  */
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	//Set a up a filter
	//Allow all messages to pass through from any ID
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x0000;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 57600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
//	__HAL_RCC_DMA1_CLK_ENABLE();
//
//	/* DMA interrupt init */
//	/* DMA1_Channel1_IRQn interrupt configuration */
//	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* Configure DMA for ADC1 */
	hdma_adc1.Instance = DMA1_Channel1; // replace if using a different channel
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_CIRCULAR; // 🔁 circular buffer
	hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
		Error_Handler();
	}

	// Link DMA to ADC
	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

	/* DMA interrupt init */
//    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
//    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	// default pins
	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	//////////////////////////////////////////
	// Gear selection pins
	/*Configure GPIO pin : Drive_Pin */
	GPIO_InitStruct.Pin = Drive_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Drive_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Neutral_Pin */
	GPIO_InitStruct.Pin = Neutral_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Reverse_Pin */
	GPIO_InitStruct.Pin = Reverse_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//////////////////////////////////////////
	// Button input pins

	GPIO_InitStruct.Pin = BrakePedal_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BrakePedal_in_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = L_SignalLight_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L_SignalLight_in_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = R_SignalLight_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(R_SignalLight_in_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = Hazard_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Hazard_in_GPIO_Port, &GPIO_InitStruct);

	//////////////////////////////////////////
	// pins providing output to lights
	GPIO_InitStruct.Pin = BrakeLight_out_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BrakeLight_out_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = L_SignalLight_out_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L_SignalLight_out_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = R_SignalLight_out_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(R_SignalLight_out_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DRLLeft_out_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DRLLeft_out_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DRLRight_out_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DRLLeft_out_GPIO_Port, &GPIO_InitStruct);

	////////////////////////////////////////

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Enable DMA interrupts after RTOS is running */
void Enable_DMA_Interrupts(void) {
	/* Give time for RTOS to stabilize */
	osDelay(100);

	/* Configure at lowest priority */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 15, 0); // Lowest priority
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	/* Activate ADC DMA with interrupts */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);

	dma_interrupts_enabled = 1;
}

/* Modified DMA IRQ Handler */
void DMA1_Channel1_IRQHandler(void) {
	if (dma_interrupts_enabled) {
		HAL_DMA_IRQHandler(&hdma_adc1);
	}
}



// Called when first half of buffer is filled
// Called when first half of adc_buf is filled (indices 0…ADC_BUF_LEN/2−1)
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
//    uint32_t sum = 0;
//    for (int i = 0; i < ADC_BUF_LEN/2; i++) {
//        sum += adc_buf[i];
//    }
//    uint32_t avg = sum / (ADC_BUF_LEN/2);
//
//    char msg[32];
//    int len = snprintf(msg, sizeof(msg), "Half avg: %lu\r\n", avg);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
//
//
//    // (Optional) If you want to signal a FreeRTOS task:
//    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    // xSemaphoreGiveFromISR(adcHalfReadySem, &xHigherPriorityTaskWoken);
//    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
//
//// Called when the entire adc_buf is filled (indices ADC_BUF_LEN/2…ADC_BUF_LEN−1)
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//    uint32_t sum = 0;
//    for (int i = ADC_BUF_LEN/2; i < ADC_BUF_LEN; i++) {
//        sum += adc_buf[i];
//    }
//    uint32_t avg = sum / (ADC_BUF_LEN/2);
//
//    char msg[32];
//    int len = snprintf(msg, sizeof(msg), "Full avg: %lu\r\n", avg);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
//
//    // (Optional) Signal the “second-half ready” if you’ve got a task waiting:
//    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    // xSemaphoreGiveFromISR(adcFullReadySem, &xHigherPriorityTaskWoken);
//    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_parked_init */
/**
 * @brief  Function implementing the parked thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_parked_init */
void parked_init(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
