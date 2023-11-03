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
#define HP203_I2C_ADDR 0x77

esp_err_t hp203b_init();
esp_err_t hp203b_read_pressure(float * pressure);
esp_err_t hp203b_read_temperature(float * temperature);
esp_err_t hp203b_read_altitude(float * altitude);

#ifdef __cpluplus
}
#endif
