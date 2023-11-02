#pragma once

#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PWM_FREQUENCY 2000
#define L_MOTOR_IO_NUM 10
#define R_MOTOR_IO_NUM 5

#define MOTOR_ADC_ATTEN ADC_ATTEN_DB_11
#define L_MOTOR_ADC_CHANNEL ADC_CHANNEL_4
#define R_MOTOR_ADC_CHANNEL ADC_CHANNEL_2

void motor_init(void);
void motor_set_duty(uint32_t duty_l, uint32_t duty_r);
void motor_get_current(int *current_l, int *current_r);

#ifdef __cplusplus
}
#endif
