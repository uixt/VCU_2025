
#include "led.h"
extern UART_HandleTypeDef huart2;

/*
 * led.c
 *
 *  Created on: Feb 12, 2025
 *      Author: 15149
 */

/* USER CODE BEGIN Header_LED_init */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_init */

  /*Configure GPIO pin : LD2_Pin */


void LED_init(void const * argument)
{
  /* USER CODE BEGIN LED_init */
  /* Infinite loop */
  //pb1 input, pc6 out

//	//turn signal code
//	//input from button, at pb1
//	  int state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
//	  if(!state){
//// output at pc6
//		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
//	        vTaskDelay(500);
//	  }
	char msg_on[] = "Button Pressed\r\n";
	char msg_off[] = "Button Released\r\n";

	for(;;) {
	    int state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	    if (!state) {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg_on, strlen(msg_on), HAL_MAX_DELAY);
	    } else {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg_off, strlen(msg_off), HAL_MAX_DELAY);
	    }
	    osDelay(500);
	}


//	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END LED_init */
}
