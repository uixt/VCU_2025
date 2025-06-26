#include "CAN_Receive.h"

// external variables
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;
extern volatile uint8_t datacheck;
extern QueueHandle_t CANq;

//Frame setup stuff, keep for final code I think
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
//char msg[64];
uint32_t val;
struct CANFrame temp;
struct CANFrame receivedFrame;

//union Data Power; 		//bus current max percent of absolute current... what is this
//union Data SendRPM; 		//motor current rpm
//union Data SendCurrent;
//union Data BusCurrent;
union Data telem_velocity_1;
union Data telem_velocity_2;
union Data avg_velocity;
union Data telem_rpm_1;
union Data telem_rpm_2;
union Data heat_sink_temp_mc1;
union Data heat_sink_temp_mc2;
union Data avg_heat_sink;

// bms
union Data bms_high_temp;
union Data bms_low_temp; // 1c
union Data bms_pack_voltage; // 0.1V
union Data bms_high_cell_voltage;
union Data bms_low_cell_voltage; //0.0001v
union Data bms_pack_current; // 0.1A
union Data serial_number;
union Data prohelion_id;


// Array sizes in bytes
int mc_msg_size = 4;
int bms_temp_size = 1;
int bms_pack_voltage_size = 2;
int bms_pack_current_size = 2;

// Rx Addresses
// CAN ID Definitions (use these in switch-case statements)
#define IDENTIFICATION_ID    0x500
#define VELOCITY_ID_MC1      0x503
#define HEATSINK_ID_MC1      0x50B
#define VELOCITY_ID_MC2      0x603
#define HEATSINK_ID_MC2      0x60B
#define BMS_TEMP_ID          0x701
#define BMS_VOLTAGE_ID       0x702
#define BMS_CURRENT_ID       0x703

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

void can_rx(void const *argument) {
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
				memcpy(telem_rpm_1.byte, RxData, mc_msg_size);
				memcpy(telem_velocity_1.byte, RxData + 5, mc_msg_size); // m/s, convert to mph
				break;

			case VELOCITY_ID_MC2:
				//type: float
				memcpy(telem_rpm_2.byte, RxData, mc_msg_size);
				memcpy(telem_velocity_2.byte, RxData + 5, mc_msg_size); // m/s, convert to mph
				break;

//				// Heat sink temperature
//			case HEATSINK_ID_MC1:
//				//type: float
//				memcpy(MotorTemp.byte, RxData, MCmsgSize);
//				memcpy(HeatSinkTempMC1.byte, RxData + 5, MCmsgSize);
//				break;
//
//			case HEATSINK_ID_MC2:
//				//type: float
//				memcpy(MotorTemp.byte, RxData, MCmsgSize);
//				memcpy(HeatSinkTempMC2.byte, RxData + 5, MCmsgSize);
//				break;

				// BMS
				// BMS Pack Temperature
			case BMS_TEMP_ID:
				//units: 1C
				memcpy(bms_high_temp.byte, RxData, bms_temp_size);
				memcpy(bms_low_temp.byte, RxData + 2, bms_temp_size);

				break;

				// BMS Pack Voltage
			case BMS_VOLTAGE_ID:
				// units: 0.0001V
				memcpy(bms_pack_voltage.byte, RxData, bms_pack_voltage_size);
				memcpy(bms_high_cell_voltage.byte, RxData + 3, bms_pack_voltage_size);
				memcpy(bms_low_cell_voltage.byte, RxData + 5, bms_pack_voltage_size);
				break;

				//Current Draw
			case BMS_CURRENT_ID:
				// units: 0.1A
				memcpy(bms_pack_current.byte, RxData, bms_pack_current_size);
				break;
			}
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			avg_velocity.f = (telem_velocity_1.f + telem_velocity_2.f) / 2.0f;



			osDelay(100);
		}

//		count = uxQueueMessagesWaiting(CANq);
	}

}

//void sendTelem(void){
//	sprintf
//	HAL_UART_Transmit()
//}
