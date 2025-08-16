#include "pca9685_servo.h"

#define PCA9685_I2C_ADDR_7B   0x40
#define PCA9685_I2C_ADDR_8B   (PCA9685_I2C_ADDR_7B << 1)

#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_LED0_ON_L     0x06
#define PCA9685_PRESCALE      0xFE

#define MODE1_RESTART         0x80
#define MODE1_AI              0x20
#define MODE1_SLEEP           0x10
#define MODE2_OUTDRV          0x04

extern I2C_HandleTypeDef hi2c1;

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

HAL_StatusTypeDef pca9685_set_pwm_freq(float freq)
{
    float prescale_f = (25000000.0f / (4096.0f * freq)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    uint8_t oldmode, sleepmode;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_I2C_ADDR_8B, PCA9685_MODE1, 1, &oldmode, 1, 10);
    sleepmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    pca9685_write8(PCA9685_MODE1, sleepmode);
    pca9685_write8(PCA9685_PRESCALE, prescale);
    pca9685_write8(PCA9685_MODE1, (oldmode | MODE1_AI) & ~MODE1_SLEEP);
    HAL_Delay(5);
    pca9685_write8(PCA9685_MODE1, (oldmode | MODE1_AI | MODE1_RESTART));
    return HAL_OK;
}

HAL_StatusTypeDef pca9685_init(void)
{
    pca9685_write8(PCA9685_MODE2, MODE2_OUTDRV);
    pca9685_write8(PCA9685_MODE1, MODE1_AI);
    HAL_Delay(5);
    return pca9685_set_pwm_freq(50.0f);
}

uint16_t servo_angle_to_ticks(float deg)
{
    if (deg < 0) deg = 0; if (deg > 180) deg = 180;
    float pulse_us = 500.0f + (deg / 180.0f) * 2000.0f;
    uint16_t ticks = (uint16_t)((pulse_us * 4096.0f) / 20000.0f + 0.5f);
    if (ticks >= 4096) ticks = 4095;
    return ticks;
}

void servo_write_deg(uint8_t ch, float deg)
{
    uint16_t off = servo_angle_to_ticks(deg);
    pca9685_write_led(ch, 0, off);
}

HAL_StatusTypeDef servo_read_deg(uint8_t ch, float* deg_out)
{
    if (!deg_out || ch > 15) return HAL_ERROR;

    // LEDn_ON/OFF 읽기
    uint8_t buf[4];
    uint16_t reg = (uint16_t)(PCA9685_LED0_ON_L + 4U * ch);
    if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_I2C_ADDR_8B, reg, 1, buf, 4, 10) != HAL_OK)
        return HAL_ERROR;

    uint16_t on  = (uint16_t)(buf[0] | ((buf[1] & 0x0F) << 8));
    uint16_t off = (uint16_t)(buf[2] | ((buf[3] & 0x0F) << 8));
    uint8_t full_on  = (buf[1] & 0x10) ? 1 : 0;  // LEDn_ON_H bit4
    uint8_t full_off = (buf[3] & 0x10) ? 1 : 0;  // LEDn_OFF_H bit4

    uint16_t width;
    if (full_on && !full_off)      width = 4096; // 항상 ON
    else if (full_off && !full_on) width = 0;    // 항상 OFF
    else                            width = (uint16_t)((off - on) & 0x0FFF);

    // prescale 읽어서 1틱 시간(µs) 계산
    uint8_t ps = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_I2C_ADDR_8B, PCA9685_PRESCALE, 1, &ps, 1, 10) != HAL_OK)
        return HAL_ERROR;

    // 1 tick = (prescale+1)/25MHz seconds
    float tick_us = ((float)(ps + 1) / 25000000.0f) * 1.0e6f;

    float pulse_us = (float)width * tick_us;

    // 500–2500 µs → 0–180°
    float deg = (pulse_us - 500.0f) * (180.0f / 2000.0f);
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;

    *deg_out = deg;
    return HAL_OK;
}
