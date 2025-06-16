/*
 * ButtonManaging.c
 *
 *  Created on: Jun 14, 2025
 *      Author: 15149
 */


#include "ButtonManaging.h"






void brake_light(void) {
	for (;;) {
		if (HAL_GPIO_ReadPin(BrakePedal_in_GPIO_Port, BrakePedal_in_Pin)
				== GPIO_PIN_SET) {
			HAL_GPIO_WritePin(BrakeLight_out_GPIO_Port, BrakeLight_out_Pin,
					GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(BrakeLight_out_GPIO_Port, BrakeLight_out_Pin,
					GPIO_PIN_RESET);

		}

	}
}

void r_signal_light(void) {
	for (;;) {
		if (HAL_GPIO_ReadPin(R_SignalLight_out_GPIO_Port,
				R_SignalLight_out_Pin) == GPIO_PIN_SET) {
			HAL_GPIO_TogglePin(R_SignalLight_out_GPIO_Port,
					R_SignalLight_out_Pin);
		}
		else {
			HAL_GPIO_WritePin(R_SignalLight_out_GPIO_Port,
					R_SignalLight_out_Pin, GPIO_PIN_RESET);
		}

	}
}


void l_signal_light(void) {
	for (;;) {
		if (HAL_GPIO_ReadPin(L_SignalLight_out_GPIO_Port,
				L_SignalLight_out_Pin) == GPIO_PIN_SET) {
			HAL_GPIO_TogglePin(L_SignalLight_out_GPIO_Port,
					L_SignalLight_out_Pin);
		}
		else {
			HAL_GPIO_WritePin(L_SignalLight_out_GPIO_Port,
					L_SignalLight_out_Pin, GPIO_PIN_RESET);
		}

	}
}


void hazard_light(void) {
	for (;;) {
		if (HAL_GPIO_ReadPin(Hazard_in_GPIO_Port, Hazard_in_Pin)
				== GPIO_PIN_SET) {
			HAL_GPIO_TogglePin(R_SignalLight_out_GPIO_Port,
					R_SignalLight_out_Pin);
			HAL_GPIO_TogglePin(L_SignalLight_out_GPIO_Port,
					L_SignalLight_out_Pin);
		} else {
			HAL_GPIO_WritePin(R_SignalLight_out_GPIO_Port, R_SignalLight_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_SignalLight_out_GPIO_Port, L_SignalLight_out_Pin, GPIO_PIN_RESET);
		}
	}

}

void buttons_100ms(void){
	brake_light();
	vTaskDelay(100);
}

void buttons_500ms(void){
	r_signal_light();
	l_signal_light();
	hazard_light();
	vTaskDelay(500);
}

//
//void signal_light(GPIO_TypeDef *port, uint16_t pin) {
//	for (;;) {
//		if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
//			HAL_GPIO_TogglePin(L_SignalLight_out_GPIO_Port,
//					L_SignalLight_out_Pin);
//			vTaskDelay(500);
//		}
//		else {
//			HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
//		}
//
//	}
//}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//    switch (GPIO_Pin) {
//    case BrakePedal_in_Pin:
//    	// if break pedal low, write pin set. else, write pin reset
//        vTaskNotifyGiveFromISR(BrakeTaskHandle, &xHigherPriorityTaskWoken);
//        break;
//
//    case L_SignalLight_in_Pin:
//        xTaskNotifyFromISR(LeftSignalTaskHandle,
//                           (uint32_t)HAL_GPIO_ReadPin(L_SignalLight_in_GPIO_Port, L_SignalLight_in_Pin),
//                           eSetValueWithOverwrite,
//                           &higherPriority);
//        portYIELD_FROM_ISR(higherPriority);
//        break;
//
//    case R_SignalLight_in_Pin:
//        vTaskNotifyGiveFromISR(RightSignalTaskHandle, &xHigherPriorityTaskWoken);
//        break;
//
//    case Hazard_in_Pin:
//        vTaskNotifyGiveFromISR(HazardTaskHandle, &xHigherPriorityTaskWoken);
//        break;
//    }
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}


//void r_signal_light(void) {
//	for (;;) {
//		if
//		HAL_GPIO_TogglePin(L_SignalLight_out_GPIO_Port, L_SignalLight_out_Pin);
//		osDelay(500);
//	}
//}
//
//void l_signal_light(void) {
//	for (;;) {
//		HAL_GPIO_TogglePin(R_SignalLight_out_GPIO_Port, R_SignalLight_out_Pin);
//		osDelay(500);
//	}
//
//}

