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
void setup(void)
{
    for (int i = 0; i < 8; ++i) HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);

    pca9685_init();            // PCA9685 50Hz 설정
    servo_write_deg(0, 90.0f); // CH0 서보를 가운데(90도)로
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
    // GPS Timeout
    // IMU Timeout
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
void loop(void)
{
    // Debug LED toggling (visual health check)
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_ms) >= 1000U) { // 1 sec toggling
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        last_ms = now;
    }
}
// ----------------------------------------------------------------
