#include "CAN_Receive.h"

// external variables
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;
extern volatile uint8_t datacheck;
extern QueueHandle_t CANq;

//Frame setup stuff, keep for final code I think
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
char msg[64];
uint32_t val;
struct CANFrame temp;
struct CANFrame receivedFrame;

//union Data Power; 		//bus current max percent of absolute current... what is this
union Data SendRPM; 		//motor current rpm
union Data SendCurrent;
//union Data BusCurrent;
union Data TelemVelocity_1;
union Data TelemVelocity_2;
union Data AvgVelocity;
union Data TelemRPM_1;
union Data TelemRPM_2;
//union Data HeatSinkTempMC1;
//union Data HeatSinkTempMC2;
//union Data AvgHeatSink;
//union Data BMS_PackTemp;
//union Data BMS_PackVoltage;
//union Data BMS_CurrentDraw;
//union Data SerialNumber;
//union Data ProhelionID;

// Array sizes in bytes
int MCmsgSize = 4;
int PackTempSize = 1;
int PackVoltageSize = 2;
int CurrentDrawSize = 2;

// Rx Addresses
// CAN ID Definitions (use these in switch-case statements)
#define IDENTIFICATION_ID    0x500
#define VELOCITY_ID_MC1      0x503
#define HEATSINK_ID_MC1      0x50B
#define VELOCITY_ID_MC2      0x603
#define HEATSINK_ID_MC2      0x60B
#define BMS_TEMP_ID          0x0
#define BMS_VOLTAGE_ID       0x0
#define BMS_CURRENT_ID       0x0

struct CANFrame makeCANFrame(CAN_RxHeaderTypeDef header, uint8_t data[8]) {
	struct CANFrame temp;

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
	temp = makeCANFrame(RxHeader, RxData);
	xQueueSendToBackFromISR(CANq, &temp, 1);

	datacheck = 1; // signal to your RTOS task
}

void StartCanRxTask(void const *argument) {
//	int count;
//	CANq = xQueueCreate(100, sizeof(struct CANFrame)); // I didn't realize this was here ca..
	vQueueAddToRegistry(CANq, "CAN queue");

	while (1) {

		if (xQueueReceiveFromISR(CANq, &receivedFrame,
				pdMS_TO_TICKS(100)) == pdPASS) {
			switch (receivedFrame.ID) {
//				case IDENTIFICATION_ID:
//					// copy first half of array///
//					//type: integer
//					memcpy(ProhelionID.byte, RxData, MCmsgSize);
//
//					// copies the second half of the array
//					memcpy(SerialNumber.byte, RxData + 5, MCmsgSize);
//
//					//case 0x403:
//					//datacheck = 1;
//					//break;
//
//					// Motor Controllers
//					// Vehicle velocity
			case VELOCITY_ID_MC1:
				//type: float
				memcpy(TelemRPM_1.byte, RxData, MCmsgSize);
				memcpy(TelemVelocity_1.byte, RxData + 5, MCmsgSize); // m/s, convert to mph
				break;

			case VELOCITY_ID_MC2:
				//type: float
				memcpy(TelemRPM_2.byte, RxData, MCmsgSize);
				memcpy(TelemVelocity_2.byte, RxData + 5, MCmsgSize); // m/s, convert to mph
				break;

//					// Heat sink temperature
//				case HEATSINK_ID_MC1:
//					//type: float
//					memcpy(MotorTemp.byte, RxData, MCmsgSize);
//					memcpy(HeatSinkTempMC1.byte, RxData + 5, MCmsgSize);
//					break;
//
//				case HEATSINK_ID_MC2:
//					//type: float
//					memcpy(MotorTemp.byte, RxData, MCmsgSize);
//					memcpy(HeatSinkTempMC2.byte, RxData + 5, MCmsgSize);
//					break;
//
//					// BMS
//					// BMS Pack Temperature
//				case BMS_TEMP_ID:
//					//type: INTEGER
//					memcpy(BMS_PackTemp.byte, RxData, PackTempSize);
//					break;
//
//					// BMS Pack Voltage
//				case BMS_VOLTAGE_ID:
//					//type: Float
//					memcpy(BMS_PackVoltage.byte, RxData, PackVoltageSize);
//					break;
//
//					//Current Draw
//				case BMS_CURRENT_ID:
//					//type: float
//					memcpy(BMS_CurrentDraw.byte, RxData, CurrentDrawSize);
//					break;
			}
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			AvgVelocity.f = (TelemVelocity_1.f + TelemVelocity_2.f) / 2.0f;
			osDelay(1000);
		}

//		count = uxQueueMessagesWaiting(CANq);
	}

}
