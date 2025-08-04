/*
 * application.c
 *
 *  Created on: May 13, 2025
 *      Author: tykim
 */

#include "main.h"
#include "stdint.h"

void setup(void)
{

}

void loop(void)
{
    // ----------------------------------------------------------------
    // 디버그 LED 토글 테스트

    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_ms) >= 1000U) { // 1초마다 토글
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        last_ms = now;
    }
    // ----------------------------------------------------------------
}
