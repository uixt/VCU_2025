//
//#include "CAN_Transmit.h"
//CAN_HandleTypeDef hcan;
//
//
//void StartCanTxTask(void const * argument)
//{
//    CAN_TxHeaderTypeDef txHeader;
//    uint8_t txData[8];
//    uint32_t txMailbox;
//
//    // Set up header
//    txHeader.IDE = CAN_ID_STD;
//    txHeader.StdId = 0x123;
//    txHeader.RTR = CAN_RTR_DATA;
//    txHeader.DLC = 2;
//
//    while (1)
//    {
//        txData[0] = 0xAB; // your payload
//        txData[1] = 0xCD;
//
//        if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
//        {
//            Error_Handler();
//        }
//
//        osDelay(1000); // send every 1 second
//    }
//}
