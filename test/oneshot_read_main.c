/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "bat.h"

const static char *TAG = "BAT_EXAMPLE";
extern adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);


void app_main(void)
{
    bat_init();

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan3_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, BAT_VOLTAGE_ADC_CHANNEL, BAT_VOLTAGE_ADC_ATTEN, &adc1_cali_chan0_handle);
    bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, BAT_CURRENT_ADC_CHANNEL, BAT_CURRENT_ADC_ATTEN, &adc1_cali_chan1_handle);
    bool do_calibration1_chan3 = example_adc_calibration_init(ADC_UNIT_1, BAT_TEMPERATURE_ADC_CHANNEL, BAT_TEMPERATURE_ADC_ATTEN, &adc1_cali_chan3_handle);


    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_VOLTAGE_ADC_CHANNEL, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BAT_VOLTAGE_ADC_CHANNEL, adc_raw[0][0]);
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BAT_VOLTAGE_ADC_CHANNEL, voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_TEMPERATURE_ADC_CHANNEL, &adc_raw[0][1]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BAT_TEMPERATURE_ADC_CHANNEL, adc_raw[0][1]);
        if (do_calibration1_chan1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BAT_TEMPERATURE_ADC_CHANNEL, voltage[0][1]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_CURRENT_ADC_CHANNEL, &adc_raw[0][2]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BAT_CURRENT_ADC_CHANNEL, adc_raw[0][2]);
        if (do_calibration1_chan3) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan3_handle, adc_raw[0][2], &voltage[0][2]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BAT_CURRENT_ADC_CHANNEL, voltage[0][2]);
        }
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_CURRENT_ADC_CHANNEL, &adc_raw[0][2]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BAT_CURRENT_ADC_CHANNEL, adc_raw[0][2]);
        if (do_calibration1_chan3) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan3_handle, adc_raw[0][2], &voltage[0][2]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BAT_CURRENT_ADC_CHANNEL, voltage[0][2]);
        }
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_CURRENT_ADC_CHANNEL, &adc_raw[0][2]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BAT_CURRENT_ADC_CHANNEL, adc_raw[0][2]);
        if (do_calibration1_chan3) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan3_handle, adc_raw[0][2], &voltage[0][2]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BAT_CURRENT_ADC_CHANNEL, voltage[0][2]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (do_calibration1_chan1) {
        example_adc_calibration_deinit(adc1_cali_chan1_handle);
    }
    if (do_calibration1_chan3) {
        example_adc_calibration_deinit(adc1_cali_chan3_handle);
    }
}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
