#include "CAN_Transmit.h"  // your own header

extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan;
#define ADC_BUF_LEN 16
extern uint16_t adc_buf[ADC_BUF_LEN];
extern union Data AvgVelocity;


//union Data {
//	int i;
//	float f;
//	uint8_t byte[8];
//};

void can_tx(void const *argument) {
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8];
	uint32_t txMailbox;
	union Data Current; //how best to control motors? Current, Velocity, mix of both?
	union Data Velocity;
	float sum = 0;
	float avg = 0;
	char msg[32]; // plenty of space for float + newline
	// Set up header
	txHeader.IDE = CAN_ID_STD;
	txHeader.StdId = 0x401;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;
	Velocity.f = 500.0f;

	while (1) {
		sum = 0;

		for (int i = 0; i < ADC_BUF_LEN; i++) {
			sum += adc_buf[i];
		}


		avg = fabs(sum / ADC_BUF_LEN);

////		snprintf(msg, sizeof(msg), "Avg: %d\r\n", (uint8_t)avg);
//		snprintf(msg, sizeof(msg), "Num: %d\r\n", (uint8_t)adc_buf[0]);
//
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//		Current.f = avg / 25600.0f;

		// 2. Convert to integer representation (0-100)
		uint8_t display_value = (uint8_t) (Current.f * 100.0f);
//
//		// 3. Print with implied decimal
		snprintf(msg, sizeof(msg), "Value: 0.%02d\r\n", display_value); // Prints 0.00-0.10

		Current.f = fabs(0.15 - avg / 25600.0f);	//low for testing purposes, maps to max 0.15
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		// gear selection
		// have to write in conditions for braking later
		//drive
		if (!HAL_GPIO_ReadPin(GPIOA, Drive_Pin) && AvgVelocity.f >= 0) {
			Velocity.f = 500.0f;
		//reverse
		} else if (!HAL_GPIO_ReadPin(GPIOA, Reverse_Pin) && AvgVelocity.f <= 0){
			Velocity.f = -500.0f;
		//neutral
		} else {
			Velocity.f = 0;
			Current.f = 0;
		}


//		Current.f = 0.00000006*(pow(avg*0.1, 2));
		txData[0] = Velocity.byte[0];
		txData[1] = Velocity.byte[1];
		txData[2] = Velocity.byte[2];
		txData[3] = Velocity.byte[3];

		txData[4] = Current.byte[0];
		txData[5] = Current.byte[1];
		txData[6] = Current.byte[2];
		txData[7] = Current.byte[3];

//		txData[0] = 0x0; // your payload
//		txData[1] = 0x0;
//		txData[2] = 0xB4;
//		txData[3] = 0x43;
//		if (!HAL_GPIO_ReadPin(Drive_GPIO_Port, Drive_Pin)) {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//		}
//
//		if (!HAL_GPIO_ReadPin(GPIOA, Neutral_Pin)) {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//		}
//
//		if (!HAL_GPIO_ReadPin(GPIOA, Reverse_Pin)) {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//		}

		//drive
//		if (!HAL_GPIO_ReadPin(Drive_GPIO_Port, Drive_Pin)) {
//			HAL_UART_Transmit(&huart2, "\nDrive\n", strlen("\nDrive\n"),
//				HAL_MAX_DELAY);
//			// Reverse
//		}
//		else if (!HAL_GPIO_ReadPin(GPIOA, Reverse_Pin)) {
//			HAL_UART_Transmit(&huart2, "\nReverse\n", strlen("\nReverse\n"),
//				HAL_MAX_DELAY);
//			// Neutral
//		}
//		else {
//			HAL_UART_Transmit(&huart2, "\nNeutral\n", strlen("\nNeutral\n"),
//				HAL_MAX_DELAY);
//		}

//		txData[0] = 0x0; // your payload
//		txData[1] = 0x0;
//		txData[2] = 0xB4;
//		txData[3] = 0x43;
//		txData[4] = 0xCD;
//		txData[5] = 0xCC;
//		txData[6] = 0x4C;
//		txData[7] = 0x3D;

		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

		if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox)
				!= HAL_OK) {
			Error_Handler();
		}
//		  HAL_UART_Transmit(&huart2, "hello world\n", 20, HAL_MAX_DELAY);

//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

		osDelay(100); // send every 1 second
	}
}
