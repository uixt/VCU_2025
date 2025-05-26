/*
 * CAN_Receive.h
 *
 *  Created on: May 23, 2025
 *      Author: 15149
 */
#include "cmsis_os.h"
#include "stm32f3xx_hal.h"
#include "initialize_pins.h"
#include <string.h>
#include <stdio.h>

#ifndef INC_CAN_RECEIVE_H_
#define INC_CAN_RECEIVE_H_

#include "main.h"

void StartCanRxTask(void const * argument);
#endif /* INC_CAN_RECEIVE_H_ */
