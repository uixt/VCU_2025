/*
 * initialize_pins.h
 *
 *  Created on: Feb 21, 2025
 *      Author: 15149
 */
#include "stm32f3xx_hal.h"

#ifndef INC_INITIALIZE_PINS_H_
#define INC_INITIALIZE_PINS_H_


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB


#endif /* INC_INITIALIZE_PINS_H_ */
