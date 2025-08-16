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

/* ADC 핸들 extern */
extern ADC_HandleTypeDef hadc1;  // VA용 (16-bit)
extern ADC_HandleTypeDef hadc3;  // TC용 (12-bit)

/* VA(ADC1) 채널 테이블: CH1~CH8 -> INP2~INP9 */
static const uint32_t VA_ADC1_CH[8] = {
    ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5,
    ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9
};

/* TC(ADC3) 채널 테이블: CH1~CH6 -> INP1,4,5,9,10,11 */
static const uint32_t TC_ADC3_CH[6] = {
    ADC_CHANNEL_1, ADC_CHANNEL_4, ADC_CHANNEL_5,
    ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11
};

/* 최근값 저장 (단위: mV) */
static uint16_t g_va_mv[8];  // 0~3300mV 가정
static uint16_t g_tc_mv[6];  // 0~3300mV 가정

/* SV 상태 저장 (0/1) */
static uint8_t  g_sv_state[8];
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

/* 공통 ADC 단일 변환: mV 반환 (VDDA=3.3V 가정) */
static uint16_t adc_read_millivolt(ADC_HandleTypeDef* hadc, uint32_t ch)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = ch;
    s.Rank         = ADC_REGULAR_RANK_1;
    /* 각 ADC의 샘플링타임은 MX_Init에서 설정된 값과 일치시키면 좋습니다. */
    if (hadc->Instance == ADC1) {
        s.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;   // ioc와 동일
        s.SingleDiff   = ADC_SINGLE_ENDED;
        s.OffsetNumber = ADC_OFFSET_NONE;
        s.Offset       = 0;
        s.OffsetSignedSaturation = DISABLE;
    } else {
        /* ADC3 */
        s.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5; // ioc와 동일
        s.SingleDiff   = ADC_SINGLE_ENDED;
        s.OffsetNumber = ADC_OFFSET_NONE;
        s.Offset       = 0;
        /* H7의 ADC3는 OffsetSign 필드가 따로 있지만 여기선 0 */
    }

    if (HAL_ADC_ConfigChannel(hadc, &s) != HAL_OK) return 0;

    if (HAL_ADC_Start(hadc) != HAL_OK) return 0;
    if (HAL_ADC_PollForConversion(hadc, 2) != HAL_OK) { HAL_ADC_Stop(hadc); return 0; }

    uint32_t raw = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    /* 해상도에 맞게 mV 변환 (VDDA=3.3V 가정) */
    if (hadc->Instance == ADC1) {
        /* 16-bit */
        return (uint16_t)((raw * 3300UL) / 65535UL);
    } else {
        /* ADC3: 12-bit */
        return (uint16_t)((raw * 3300UL) / 4095UL);
    }
}

/* SV 상태 읽기: 현재 핀 입력 레지스터 값(IDR) 기반 (출력이어도 읽기 가능) */
static void read_sv_states(uint8_t out[8])
{
    for (int i = 0; i < 8; ++i) {
        out[i] = (HAL_GPIO_ReadPin(SV_PORTS[i], SV_PINS[i]) == GPIO_PIN_SET) ? 1 : 0;
    }
}

/* VA 8채널 스캔 (ADC1) */
static void read_va_all_mv(uint16_t out_mv[8])
{
    for (int i = 0; i < 8; ++i) {
        out_mv[i] = adc_read_millivolt(&hadc1, VA_ADC1_CH[i]);
    }
}

/* TC 6채널 스캔 (ADC3) — 여기서는 mV까지만, 온도로의 변환은 센서/게인 정보 필요 */
static void read_tc_all_mv(uint16_t out_mv[6])
{
    for (int i = 0; i < 6; ++i) {
        out_mv[i] = adc_read_millivolt(&hadc3, TC_ADC3_CH[i]);
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

    // === 100ms마다 SV/VA/TC 갱신 ===
    static uint32_t sense_t = 0;
    if (now - sense_t >= 100U) {
        read_sv_states(g_sv_state);
        read_va_all_mv(g_va_mv);
        read_tc_all_mv(g_tc_mv);
        sense_t = now;
    }

    // === 100ms마다 UMB로 상태 요약 전송 ===
    static uint32_t rep_t = 0;
    if (now - rep_t >= 100U) {
        char msg[256];
        int n = snprintf(msg, sizeof(msg),
            "LEN UMB=%u TLM=%u IMU=%u GPS=%u | "
            "SV=%u%u%u%u%u%u%u%u | "
            "VA(mV)=%u,%u,%u,%u,%u,%u,%u,%u | "
            "TC(mV)=%u,%u,%u,%u,%u,%u\r\n",
            (unsigned)g_uart[UART_CH_UMB].len,
            (unsigned)g_uart[UART_CH_TLM].len,
            (unsigned)g_uart[UART_CH_IMU].len,
            (unsigned)g_uart[UART_CH_GPS].len,
            g_sv_state[0], g_sv_state[1], g_sv_state[2], g_sv_state[3],
            g_sv_state[4], g_sv_state[5], g_sv_state[6], g_sv_state[7],
            g_va_mv[0], g_va_mv[1], g_va_mv[2], g_va_mv[3],
            g_va_mv[4], g_va_mv[5], g_va_mv[6], g_va_mv[7],
            g_tc_mv[0], g_tc_mv[1], g_tc_mv[2],
            g_tc_mv[3], g_tc_mv[4], g_tc_mv[5]
        );
        if (n > 0) {
            HAL_UART_Transmit(g_uart[UART_CH_UMB].huart, (uint8_t*)msg, (uint16_t)n, 10);
        }
        rep_t = now;
    }
}
// ----------------------------------------------------------------
