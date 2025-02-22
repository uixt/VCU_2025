
#include "led.h"
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
  for(;;)
  {
	  int state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(!state){
		  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		  HAL_Delay(500);
	  }else{
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	        vTaskDelay(500);
	  }
  }
  /* USER CODE END LED_init */
}
