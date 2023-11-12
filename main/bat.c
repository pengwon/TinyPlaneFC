#include "bat.h"

extern adc_oneshot_unit_handle_t adc1_handle;

void bat_init(void)
{
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
