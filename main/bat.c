#include "bat.h"

adc_oneshot_unit_handle_t adc1_handle;

void bat_init(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = BAT_VOLTAGE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_VOLTAGE_ADC_CHANNEL, &config));

    config.atten = BAT_CURRENT_ADC_ATTEN;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_CURRENT_ADC_CHANNEL, &config));

    config.atten = BAT_TEMPERATURE_ADC_ATTEN;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_TEMPERATURE_ADC_CHANNEL, &config));
}

int bat_get_voltage(void)
{
    int voltage = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_VOLTAGE_ADC_CHANNEL, &voltage));
    return voltage;
}

int bat_get_current(void)
{
    int current = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_CURRENT_ADC_CHANNEL, &current));
    return current;
}

int bat_get_temperature(void)
{
    int temperature = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_TEMPERATURE_ADC_CHANNEL, &temperature));
    return temperature;
}
