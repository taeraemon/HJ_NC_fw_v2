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

// #include "i2c.h"          // hi2c1/hi2c2 등 사용 중인 I2C 핸들
#include <stdio.h>        // snprintf
extern I2C_HandleTypeDef hi2c1;
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



// ----------------------------------------------------------------
// USB가 제대로 붙었을 때만 전송
static inline uint8_t cdc_send_line(const char* s)
{
    extern USBD_HandleTypeDef hUsbDeviceHS;
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return USBD_FAIL;
    return CDC_Transmit_HS((uint8_t*)s, (uint16_t)strlen(s));
}

static void I2C_ScanOnce(I2C_HandleTypeDef* hi2c)
{
    char line[48];
    cdc_send_line("\r\n[I2C scan]\r\n");

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        // HAL은 8-bit 주소를 받으므로 7-bit << 1
        if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 1, 2) == HAL_OK) {
            int n = snprintf(line, sizeof(line), "  device @ 0x%02X\r\n", addr);
            if (n > 0) cdc_send_line(line);
            found++;
        }
    }
    if (!found) cdc_send_line("  (none)\r\n");
}
// ----------------------------------------------------------------





// ===== PCA9685 간단 드라이버 =====
#define PCA9685_I2C_ADDR_7B   0x40            // A0~A5=GND 기본
#define PCA9685_I2C_ADDR_8B   (PCA9685_I2C_ADDR_7B << 1)

#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_LED0_ON_L     0x06
#define PCA9685_PRESCALE      0xFE

// MODE1 bits
#define MODE1_RESTART         0x80
#define MODE1_AI              0x20
#define MODE1_SLEEP           0x10
// MODE2 bits
#define MODE2_OUTDRV          0x04   // Totem-pole

static HAL_StatusTypeDef pca9685_write8(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDR_8B, reg, 1, &val, 1, 10);
}

static HAL_StatusTypeDef pca9685_write_led(uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t buf[4] = { on & 0xFF, on >> 8, off & 0xFF, off >> 8 };
    return HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDR_8B,
                             (uint16_t)(PCA9685_LED0_ON_L + 4*ch),
                             1, buf, 4, 10);
}

// freq: Hz (예: 50)
static HAL_StatusTypeDef pca9685_set_pwm_freq(float freq)
{
    // prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_f = (25000000.0f / (4096.0f * freq)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    uint8_t oldmode, sleepmode;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_I2C_ADDR_8B, PCA9685_MODE1, 1, &oldmode, 1, 10);
    sleepmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;                 // SLEEP=1
    pca9685_write8(PCA9685_MODE1, sleepmode);
    pca9685_write8(PCA9685_PRESCALE, prescale);                           // prescale 설정
    pca9685_write8(PCA9685_MODE1, (oldmode | MODE1_AI) & ~MODE1_SLEEP);   // AI=1, SLEEP=0
    HAL_Delay(5);
    pca9685_write8(PCA9685_MODE1, (oldmode | MODE1_AI | MODE1_RESTART));  // RESTART
    return HAL_OK;
}

static HAL_StatusTypeDef pca9685_init(void)
{
    // MODE2: OUTDRV, MODE1: AI 켜고 sleep 해제
    pca9685_write8(PCA9685_MODE2, MODE2_OUTDRV);
    pca9685_write8(PCA9685_MODE1, MODE1_AI);   // SLEEP=0 상태
    HAL_Delay(5);
    return pca9685_set_pwm_freq(50.0f);        // 서보용 50Hz
}

// 각도(0~180) → 펄스폭(1000~2000us) → 12bit tick
static uint16_t servo_angle_to_ticks(float deg)
{
    if (deg < 0) deg = 0; if (deg > 180) deg = 180;
    // float pulse_us = 1000.0f + (deg / 180.0f) * 1000.0f; // 1.0~2.0 ms
    float pulse_us = 500.0f + (deg / 180.0f) * 2000.0f;  // 0.5~2.5ms
    // ticks = pulse_us / 20000 * 4096
    uint16_t ticks = (uint16_t)((pulse_us * 4096.0f) / 20000.0f + 0.5f);
    if (ticks >= 4096) ticks = 4095;
    return ticks;
}

static void servo_write_deg(uint8_t ch, float deg)
{
    uint16_t off = servo_angle_to_ticks(deg);
    pca9685_write_led(ch, 0, off);   // on=0, off=ticks
}



void setup(void)
{
    for (int i = 0; i < 8; ++i) HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);

    pca9685_init();            // PCA9685 50Hz 설정
    servo_write_deg(0, 90.0f); // CH0 서보를 가운데(90도)로
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



    // ----------------------------------------------------------------
    // I2C 스캔 테스트

    static uint32_t i2c_scan_t = 0;
    if (now - i2c_scan_t >= 5000U) {              // 5초마다 스캔
        I2C_ScanOnce(&hi2c1);                     // 사용 중인 핸들로 변경 가능
        i2c_scan_t = now;
    }
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // 서보 테스트: 1초마다 0 ↔ 180°
    static uint32_t servo_t = 0;
    static int dir = 0;         // 0: 0도, 1: 180도
    if (now - servo_t >= 5000U) {
        servo_write_deg(0, dir ? 180.0f : 0.0f);
        dir ^= 1;
        servo_t = now;
    }
    // ----------------------------------------------------------------
}
