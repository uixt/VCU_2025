#include "CAN_Transmit.h"  // your own header

extern CAN_HandleTypeDef hcan;

void StartCanTxTask(void const *argument) {
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8];
	uint32_t txMailbox;

	// Set up header
	txHeader.IDE = CAN_ID_STD;
	txHeader.StdId = 0x401;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;

	while (1) {
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

		txData[0] = 0x0; // your payload
		txData[1] = 0x0;
		txData[2] = 0xB4;
		txData[3] = 0x43;
		txData[4] = 0xCD;
		txData[5] = 0xCC;
		txData[6] = 0x4C;
		txData[7] = 0x3D;

		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

		if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox)
				!= HAL_OK) {
			Error_Handler();
		}
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);


		osDelay(100); // send every 1 second
	}
}
