/*
 * application.c
 *
 *  Created on: May 13, 2025
 *      Author: tykim
 */





// ----------------------------------------------------------------
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>        // snprintf

#include "pca9685_servo.h"
// ----------------------------------------------------------------





// ----------------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2;   // UMB
extern UART_HandleTypeDef huart5;   // TLM
extern UART_HandleTypeDef huart4;   // IMU
extern UART_HandleTypeDef huart3;   // GPS

#define UMB_RX_BUF_SIZE 1024
static uint8_t        g_umb_rx_buf[UMB_RX_BUF_SIZE];
static volatile uint16_t g_umb_len = 0;        // 현재 쌓인 길이
static uint8_t        g_umb_rx_byte;           // IRQ 1바이트 버퍼

static GPIO_TypeDef* const SV_PORTS[8] = {
    SV_CH1_GPIO_Port, SV_CH2_GPIO_Port, SV_CH3_GPIO_Port, SV_CH4_GPIO_Port,
    SV_CH5_GPIO_Port, SV_CH6_GPIO_Port, SV_CH7_GPIO_Port, SV_CH8_GPIO_Port
};
static const uint16_t SV_PINS[8] = {
    SV_CH1_Pin, SV_CH2_Pin, SV_CH3_Pin, SV_CH4_Pin,
    SV_CH5_Pin, SV_CH6_Pin, SV_CH7_Pin, SV_CH8_Pin
};
// ----------------------------------------------------------------



/* >>> ADD: UART 수신 콜백 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {             // UMB
        if (g_umb_len < UMB_RX_BUF_SIZE) {
            g_umb_rx_buf[g_umb_len++] = g_umb_rx_byte;
        } else {
            // "다 채우면 비우고 쌓고": 길이 0으로 리셋(원하면 memset으로 실제 비움)
            g_umb_len = 0;
            // memset(g_umb_rx_buf, 0, UMB_RX_BUF_SIZE); // 진짜 초기화가 필요하면 주석 해제
            g_umb_rx_buf[g_umb_len++] = g_umb_rx_byte;  // 첫 바이트 저장
        }
        // 다음 1바이트 수신 재개
        HAL_UART_Receive_IT(&huart2, &g_umb_rx_byte, 1);
    }
}

/* 수신에러 발생 시 재개 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // 간단 재시작
        HAL_UART_Receive_IT(&huart2, &g_umb_rx_byte, 1);
    }
}
/* <<< ADD END */



// ----------------------------------------------------------------
void setup(void)
{
    for (int i = 0; i < 8; ++i) HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);

    pca9685_init();            // PCA9685 50Hz 설정
    servo_write_deg(0, 90.0f); // CH0 서보를 가운데(90도)로

    HAL_UART_Receive_IT(&huart2, &g_umb_rx_byte, 1);
}
// ----------------------------------------------------------------
// State Definition
    // SV
    // VA
    // T_CJ
    // TC
    // IMU
    // GPS
    // Servo
    // Volt, Amp

// Fault Definition
    // UMB Timeout
    // TLM Timeout
    // IMU Timeout
    // GPS Timeout
    // Servo Timeout
// ----------------------------------------------------------------
// Timer : State Update
    // Update State
    // Diagnose Fault

// Timer : State Report
    // To UMB, TLM

// Timer : Logging
    // if logging actiavted,
        // Save Current State to SD Card

// UART Receive
    // if UMB received, || if TLM received,
        // Save to each Buffer
    // if IMU received, || if GPS received,
        // encode with library

// UART Execute
    // if UMB received, || if TLM received,
        // Execute Command
// ----------------------------------------------------------------
/*
GCS -> NC
- SV Command
:몇번째;온오프#

- Servo Command
:몇번쨰;각도#



NC -> GCS
:
SV (CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8)
;
VA (CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8)
;
T_CJ
;
TC (CH1,CH2,CH3,CH4,CH5,CH6)
;
SERVO (CH1,CH2,CH3,CH4)
;
IMU (PITCH, ROLL, YAW, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z)
;
GPS (FIX_READY, LAT, LON, ALT, SPEED, HEADING, SATELLITE COUNT)
;
STATE (Volt, Amp)
;
FAULT (UMB_TIMEOUT, TLM_TIMEOUT, GPS_TIMEOUT, IMU_TIMEOUT, SERVO_TIMEOUT)
#
*/
// ----------------------------------------------------------------
void loop(void)
{
    // Debug LED toggling (visual health check)
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_ms) >= 1000U) { // 1 sec toggling
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        last_ms = now;
    }

    /* >>> ADD: 1초마다 UMB 버퍼 길이 보고 */
    static uint32_t umb_report_t = 0;
    if (now - umb_report_t >= 1000U) {
        uint16_t len_snapshot = g_umb_len; // IRQ와 경합 대비 스냅샷만 읽음
        char msg[48];
        int n = snprintf(msg, sizeof(msg), "UMB_RX_LEN=%u\r\n", (unsigned)len_snapshot);
        if (n > 0) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)n, 10); // 블로킹 TX
        }
        umb_report_t = now;
    }
    /* <<< ADD END */
}
// ----------------------------------------------------------------
