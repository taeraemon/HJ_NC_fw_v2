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

typedef enum {
    UART_CH_UMB = 0,  // USART2
    UART_CH_TLM,      // UART5
    UART_CH_IMU,      // UART4
    UART_CH_GPS,      // USART3
    UART_CH_COUNT
} UartCh;
  
// 채널별 RX 상태
typedef struct {
    UART_HandleTypeDef* huart;   // HAL 핸들 포인터
    uint8_t*            buf;     // 앱 RX 버퍼
    uint16_t            cap;     // 버퍼 용량
    volatile uint16_t   len;     // 현재 쌓인 길이
    uint8_t             rx1;     // 1바이트 임시 저장 (IRQ용)
} UartRx;

extern UART_HandleTypeDef huart2;   // UMB
extern UART_HandleTypeDef huart5;   // TLM
extern UART_HandleTypeDef huart4;   // IMU
extern UART_HandleTypeDef huart3;   // GPS

#define UMB_RX_BUF_SIZE 1024
#define TLM_RX_BUF_SIZE 1024
#define IMU_RX_BUF_SIZE 1024
#define GPS_RX_BUF_SIZE 1024

static uint8_t umb_buf[UMB_RX_BUF_SIZE];
static uint8_t tlm_buf[TLM_RX_BUF_SIZE];
static uint8_t imu_buf[IMU_RX_BUF_SIZE];
static uint8_t gps_buf[GPS_RX_BUF_SIZE];

// 채널 테이블
static UartRx g_uart[UART_CH_COUNT] = {
    [UART_CH_UMB] = { &huart2, umb_buf, UMB_RX_BUF_SIZE, 0, 0 },
    [UART_CH_TLM] = { &huart5, tlm_buf, TLM_RX_BUF_SIZE, 0, 0 },
    [UART_CH_IMU] = { &huart4, imu_buf, IMU_RX_BUF_SIZE, 0, 0 },
    [UART_CH_GPS] = { &huart3, gps_buf, GPS_RX_BUF_SIZE, 0, 0 },
};


static GPIO_TypeDef* const SV_PORTS[8] = {
    SV_CH1_GPIO_Port, SV_CH2_GPIO_Port, SV_CH3_GPIO_Port, SV_CH4_GPIO_Port,
    SV_CH5_GPIO_Port, SV_CH6_GPIO_Port, SV_CH7_GPIO_Port, SV_CH8_GPIO_Port
};
static const uint16_t SV_PINS[8] = {
    SV_CH1_Pin, SV_CH2_Pin, SV_CH3_Pin, SV_CH4_Pin,
    SV_CH5_Pin, SV_CH6_Pin, SV_CH7_Pin, SV_CH8_Pin
};
// ----------------------------------------------------------------





// ----------------------------------------------------------------
// 공통 처리 루틴
static inline void uart_rx_byte_push(UartRx* u)
{
  if (u->len < u->cap) {
    u->buf[u->len++] = u->rx1;
  } else {
    // 정책: 꽉 차면 비우고 처음부터 (필요하면 memset으로 지우기)
    u->len = 0;
    // memset(u->buf, 0, u->cap);
    u->buf[u->len++] = u->rx1;
  }
}

// HAL 수신 완료 콜백
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 어떤 채널인지 매칭
    for (int ch = 0; ch < UART_CH_COUNT; ++ch) {
        if (huart == g_uart[ch].huart) {
            uart_rx_byte_push(&g_uart[ch]);
            // 다음 바이트 수신 재개
            HAL_UART_Receive_IT(g_uart[ch].huart, &g_uart[ch].rx1, 1);
            break;
        }
    }
}

// 에러 콜백: 수신 재개
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (int ch = 0; ch < UART_CH_COUNT; ++ch) {
        if (huart == g_uart[ch].huart) {
            HAL_UART_Receive_IT(g_uart[ch].huart, &g_uart[ch].rx1, 1);
            break;
        }
    }
}
// ----------------------------------------------------------------





// ----------------------------------------------------------------
void setup(void)
{
    for (int i = 0; i < 8; ++i) HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);

    pca9685_init();            // PCA9685 50Hz 설정
    servo_write_deg(0, 90.0f); // CH0 서보를 가운데(90도)로

    // 모든 UART 채널 1바이트 IRQ 수신 시작
    for (int ch = 0; ch < UART_CH_COUNT; ++ch) {
        g_uart[ch].len = 0;
        HAL_UART_Receive_IT(g_uart[ch].huart, &g_uart[ch].rx1, 1);
    }
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
    uint32_t now = HAL_GetTick();
    
    // Debug LED toggling (visual health check)
    static uint32_t last_ms = 0;
    if ((now - last_ms) >= 1000U) { // 1 sec toggling
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        last_ms = now;
    }


    static uint32_t rep_t = 0;
    // 100ms마다 각 채널 len 보고 (UMB로 송신)
    if (now - rep_t >= 100) {
        char msg[96];
        // 스냅샷(IRQ 경합 최소화 위해 읽기만)
        uint16_t l_umb = g_uart[UART_CH_UMB].len;
        uint16_t l_tlm = g_uart[UART_CH_TLM].len;
        uint16_t l_imu = g_uart[UART_CH_IMU].len;
        uint16_t l_gps = g_uart[UART_CH_GPS].len;

        int n = snprintf(msg, sizeof(msg),
                        "LEN UMB=%u TLM=%u IMU=%u GPS=%u\r\n",
                        (unsigned)l_umb, (unsigned)l_tlm,
                        (unsigned)l_imu, (unsigned)l_gps);
        if (n > 0) {
            HAL_UART_Transmit(g_uart[UART_CH_UMB].huart, (uint8_t*)msg, (uint16_t)n, 10);
        }
        rep_t = now;
    }
}
// ----------------------------------------------------------------
