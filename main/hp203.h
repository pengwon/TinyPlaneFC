#pragma once

#ifdef __cpluplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "driver/i2c.h"

#define HP203_I2C_PORT I2C_NUM_0
#define HP203_I2C_ADDRESS 0x77
#define HP203_I2C_TIME_OUT 100
#define HP203_CONVERT_TIME 128

#define HP203_INT_PA_RDY_EN (uint8_t) BIT5
#define HP203_INT_T_RDY_EN (uint8_t) BIT4


typedef enum {
    HP203_CONVERT_PT = 0b00,
    HP203_CONVERT_T = 0b10
} hp203_convert_channel_t;

typedef enum {
    HP203_CONVERT_OSR_4096 = 0b000,
    HP203_CONVERT_OSR_2048 = 0b001,
    HP203_CONVERT_OSR_1024 = 0b010,
    HP203_CONVERT_OSR_512 = 0b011,
    HP203_CONVERT_OSR_256 = 0b100,
    HP203_CONVERT_OSR_128 = 0b101
} hp203_convert_osr_t;

typedef void *hp203_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param port I2C port number
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
hp203_handle_t hp203_create(i2c_port_t port, const uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of hp203
 */
void hp203_delete(hp203_handle_t sensor);
esp_err_t hp203_reset(hp203_handle_t sensor);
esp_err_t hp203_start_convert(hp203_handle_t sensor, hp203_convert_channel_t channel, hp203_convert_osr_t osr);
esp_err_t hp203_read_pressure(hp203_handle_t sensor, float *pressure);
esp_err_t hp203_read_temperature(hp203_handle_t sensor, float *temperature);
esp_err_t hp203_read_altitude(hp203_handle_t sensor, float *altitude);
esp_err_t hp203_read_pt(hp203_handle_t sensor, float *pressure, float *temperature);
esp_err_t hp203_read_at(hp203_handle_t sensor, float *altitude, float *temperature);
esp_err_t hp203_int_enable(hp203_handle_t sensor, uint8_t int_en);
bool hp203_dev_ready(hp203_handle_t sensor);
bool hp203_pa_ready(hp203_handle_t sensor);
bool hp203_t_ready(hp203_handle_t sensor);


#ifdef __cpluplus
}
#endif
