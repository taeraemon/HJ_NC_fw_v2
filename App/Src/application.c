/*
 * application.c
 *
 *  Created on: May 13, 2025
 *      Author: tykim
 */

#include "main.h"

void setup(void)
{

}

void loop(void)
{
	// LED 켜기
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(500);  // 500ms 대기
	
	// LED 끄기
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(500);  // 500ms 대기
}
