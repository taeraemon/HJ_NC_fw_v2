/*
 * application.c
 *
 *  Created on: May 13, 2025
 *      Author: tykim
 */



// ----------------------------------------------------------------
#include "main.h"
#include <stdint.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
// ----------------------------------------------------------------



// ----------------------------------------------------------------
static GPIO_TypeDef* const SV_PORTS[8] = {
    SV_CH1_GPIO_Port, SV_CH2_GPIO_Port, SV_CH3_GPIO_Port, SV_CH4_GPIO_Port,
    SV_CH5_GPIO_Port, SV_CH6_GPIO_Port, SV_CH7_GPIO_Port, SV_CH8_GPIO_Port
};
static const uint16_t SV_PINS[8] = {
    SV_CH1_Pin, SV_CH2_Pin, SV_CH3_Pin, SV_CH4_Pin,
    SV_CH5_Pin, SV_CH6_Pin, SV_CH7_Pin, SV_CH8_Pin
};
// ----------------------------------------------------------------





void setup(void)
{
    for (int i = 0; i < 8; ++i) HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);
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



    // ----------------------------------------------------------------
    // SV 테스트

    static uint32_t sv_t = 0;
    static uint8_t idx = 0;

    if (now - sv_t >= 1000U) {
        // 모두 끄고 현재 것만 켜기
        for (int i = 0; i < 8; ++i) {
            HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(SV_PORTS[idx], SV_PINS[idx], GPIO_PIN_SET);

        idx = (idx + 1) & 0x07;   // 0~7 반복
        sv_t = now;
    }
    // ----------------------------------------------------------------


    // ----------------------------------------------------------------
    // USB 문자열 전송 테스트

    static uint32_t usb_t = 0;
    if (now - usb_t >= 1000U) {
        const char *msg = "hello\r\n";
        if (CDC_Transmit_HS((uint8_t*)msg, (uint16_t)strlen(msg)) == USBD_OK) {
            usb_t = now;  // 성공 시에만 타임스탬프 갱신
        }
        // USBD_BUSY면 다음 loop에서 다시 시도 (논블로킹)
    }
    // ----------------------------------------------------------------
}
