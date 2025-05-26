

#include "CAN_Receive.h"  // your own header

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;
extern volatile uint8_t datacheck;

void StartCanRxTask(void const * argument)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    char msg[64];
    uint32_t val;

    while (1)
    {
//    	val = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
//        if (val > 0)
//        {
//            if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
//            {
//            	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
//                sprintf(msg, "RX CAN ID: 0x%X Data: %02X %02X\r\n", rxHeader.StdId, rxData[0], rxData[1]);
//                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//            }
//        }

        osDelay(10); // small delay to avoid hogging CPU
    }
}


//
//void StartCanRxTask(void const * argument)
//{
//    CAN_RxHeaderTypeDef rxHeader;
//    uint8_t rxData[8];
//    char msg[64];
//
//    while (1)
//    {
//        if (datacheck)
//        {
//            if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
//            {
//                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
//                sprintf(msg, "RX CAN ID: 0x%X Data: %02X %02X\r\n", rxHeader.StdId, rxData[0], rxData[1]);
//                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//            }
//
//            datacheck = 0;
//        }
//
//        osDelay(10);
//    }
//}

