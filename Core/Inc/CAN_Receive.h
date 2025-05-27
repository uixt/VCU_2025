/*
 * CAN_Receive.h
 *
 *  Created on: May 23, 2025
 *      Author: 15149
 */
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "stm32f3xx_hal.h"
#include "initialize_pins.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#ifndef INC_CAN_RECEIVE_H_
#define INC_CAN_RECEIVE_H_

extern volatile struct CANframe {
		uint32_t ID;
    	uint8_t rxData[8];
};

void StartCanRxTask(void const * argument);
#endif /* INC_CAN_RECEIVE_H_ */
