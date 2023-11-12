#pragma once

#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BAT_VOLTAGE_ADC_ATTEN ADC_ATTEN_DB_11
#define BAT_CURRENT_ADC_ATTEN ADC_ATTEN_DB_0
#define BAT_TEMPERATURE_ADC_ATTEN ADC_ATTEN_DB_11
#define BAT_VOLTAGE_ADC_CHANNEL ADC_CHANNEL_0
#define BAT_CURRENT_ADC_CHANNEL ADC_CHANNEL_3
#define BAT_TEMPERATURE_ADC_CHANNEL ADC_CHANNEL_1

typedef struct {
    int voltage;
    int current;
    int temperature;
} bat_data_t;

void bat_init(void);
int bat_get_voltage(void);
int bat_get_current(void);
int bat_get_temperature(void); 

#ifdef __cplusplus
}
#endif
