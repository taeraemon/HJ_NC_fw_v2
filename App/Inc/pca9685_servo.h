#ifndef __PCA9685_SERVO_H__
#define __PCA9685_SERVO_H__

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef pca9685_init(void);
HAL_StatusTypeDef pca9685_set_pwm_freq(float freq);
void servo_write_deg(uint8_t ch, float deg);
uint16_t servo_angle_to_ticks(float deg);
HAL_StatusTypeDef servo_read_deg(uint8_t ch, float* deg_out);

#endif // __PCA9685_SERVO_H__
