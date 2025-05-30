#include "CAN_Receive.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;
extern volatile uint8_t datacheck;

extern QueueHandle_t CANq;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
char msg[64];
uint32_t val;
struct CANframe temp;
struct CANframe receivedFrame;

struct CANframe makeFrame(CAN_RxHeaderTypeDef header, uint8_t data[8]) {
	struct CANframe temp;

	temp.ID = header.StdId;
	temp.rxData[0] = data[0];
	temp.rxData[1] = data[1];
	temp.rxData[2] = data[2];
	temp.rxData[3] = data[3];
	temp.rxData[4] = data[4];
	temp.rxData[5] = data[5];
	temp.rxData[6] = data[6];
	temp.rxData[7] = data[7];

	return temp;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		Error_Handler();
	}
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	temp = makeFrame(RxHeader, RxData);
	xQueueSendToBackFromISR(CANq, &temp, 1);

	datacheck = 1; // signal to your RTOS task
}

void StartCanRxTask(void const *argument) {
	int count;
	CANq = xQueueCreate(100, sizeof(struct CANframe));
	vQueueAddToRegistry(CANq, "CAN queue");

	while (1) {

		if (xQueueReceiveFromISR(CANq, &receivedFrame, pdMS_TO_TICKS(100)) == pdPASS) {

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			osDelay(1000);
		}

		count = uxQueueMessagesWaiting(CANq);
	}

}
