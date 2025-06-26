/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "CAN_Transmit.h"
#include "CAN_Receive.h"
#include "ButtonManaging.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* USER CODE BEGIN EFP */
union Data {
	int i;
	float f;
	uint8_t byte[8];
};




/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB


// Gear selection pins
#define Drive_Pin GPIO_PIN_7
#define Drive_GPIO_Port GPIOA

#define Neutral_Pin GPIO_PIN_6
#define Neutral_GPIO_Port GPIOA

#define Reverse_Pin GPIO_PIN_5
#define Reverse_GPIO_Port GPIOA


// button input pins
#define BrakePedal_in_Pin GPIO_PIN_9
#define BrakePedal_in_GPIO_Port GPIOA

#define L_SignalLight_in_Pin GPIO_PIN_10
#define L_SignalLight_in_GPIO_Port GPIOB

#define R_SignalLight_in_Pin GPIO_PIN_1
#define R_SignalLight_in_GPIO_Port GPIOB

#define Hazard_in_Pin GPIO_PIN_0
#define Hazard_in_GPIO_Port GPIOC


// pins providing output to lights
#define BrakeLight_out_Pin GPIO_PIN_8
#define BrakeLight_out_GPIO_Port GPIOC

#define L_SignalLight_out_Pin GPIO_PIN_11
#define L_SignalLight_out_GPIO_Port GPIOB

#define R_SignalLight_out_Pin GPIO_PIN_6
#define R_SignalLight_out_GPIO_Port GPIOC

#define DRLLeft_out_Pin GPIO_PIN_5
#define DRLLeft_out_GPIO_Port GPIOC

#define DRLRight_out_Pin GPIO_PIN_12
#define DRLRight_out_GPIO_Port GPIOB




#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
