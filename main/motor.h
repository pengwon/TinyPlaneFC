#pragma once

#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PWM_FREQUENCY 2000
#define MOTOR_PWM_DUTY_MAX 8191
#define MOTOR_PWM_DUTY_MIN 0
#define L_MOTOR_IO_NUM 10
#define R_MOTOR_IO_NUM 5

#define MOTOR_ADC_ATTEN ADC_ATTEN_DB_11
#define MOTOR_ADC_VREF 1100
#define L_MOTOR_ADC_CHANNEL ADC_CHANNEL_4
#define R_MOTOR_ADC_CHANNEL ADC_CHANNEL_2

#define MOTOR_CURRENT_MAX 100
#define MOTOR_CURRENT_MIN 0
#define MOTOR_THROTTLE_MAX 2000
#define MOTOR_THROTTLE_MIN 1000
#define MOTOR_KP 10
#define MOTOR_KI 0.1
#define MOTOR_KD 0.1



typedef struct {
    uint32_t duty_l;
    uint32_t duty_r;
    int32_t current_l;
    int32_t current_r;
} motor_data_t;

void motor_init(void);
void motor_init_pid(void);
void motor_update_current_offset(void);
void motor_set_frequency(uint32_t frequency);
void motor_set_duty(uint32_t duty_l, uint32_t duty_r);
void motor_get_current(int *current_l, int *current_r);
void motor_set_throttle(int throttle_l, int throttle_r);

#ifdef __cplusplus
}
#endif
