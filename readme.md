Inspection demo





``` c
#include "usb_device.h"
#include "usbd_cdc_if.h"

// ----------------------------------------------------------------
// USB가 제대로 붙었을 때만 전송
static inline uint8_t cdc_send_line(const char* s)
{
    extern USBD_HandleTypeDef hUsbDeviceHS;
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return USBD_FAIL;
    return CDC_Transmit_HS((uint8_t*)s, (uint16_t)strlen(s));
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
```


``` c
extern I2C_HandleTypeDef hi2c1;

// ----------------------------------------------------------------
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

// ----------------------------------------------------------------
// I2C 스캔 테스트
static uint32_t i2c_scan_t = 0;
if (now - i2c_scan_t >= 5000U) {              // 5초마다 스캔
    I2C_ScanOnce(&hi2c1);                     // 사용 중인 핸들로 변경 가능
    i2c_scan_t = now;
}
// ----------------------------------------------------------------
```



``` c
extern I2C_HandleTypeDef hi2c1;

pca9685_init();            // PCA9685 50Hz 설정
servo_write_deg(0, 90.0f); // CH0 서보를 가운데(90도)로

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
```





``` c
for (int i = 0; i < 8; ++i) HAL_GPIO_WritePin(SV_PORTS[i], SV_PINS[i], GPIO_PIN_RESET);

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

```






``` c
// ----------------------------------------------------------------
// 디버그 LED 토글 테스트

static uint32_t last_ms = 0;
uint32_t now = HAL_GetTick();

if ((now - last_ms) >= 1000U) { // 1초마다 토글
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    last_ms = now;
}
// ----------------------------------------------------------------
```





``` c
extern UART_HandleTypeDef huart2;   // UMB

// ----------------------------------------------------------------
// UART2 문자열 전송 테스트: 1초마다 "hello, <ms>"
static uint32_t uart2_t = 0;
if (now - uart2_t >= 10U) {
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "hello, %lu\r\n", (unsigned long)now);
    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 10);
    }
    uart2_t = now;
}
// ----------------------------------------------------------------
```
