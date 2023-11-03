#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"

#include "hp203.h"

const static char *TAG = "HP203_EXAMPLE";

#define I2C_MASTER_SCL_IO 8         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 9         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    float tmp, pressure, temperature, altitude[4], avg;
    int i = 0;
    i2c_master_init();
    hp203b_init();

    ESP_ERROR_CHECK(hp203b_read_altitude(&altitude[0]));
    ESP_ERROR_CHECK(hp203b_read_altitude(&altitude[1]));
    ESP_ERROR_CHECK(hp203b_read_altitude(&altitude[2]));
    ESP_ERROR_CHECK(hp203b_read_altitude(&altitude[3]));
    avg = (altitude[0] + altitude[1] + altitude[2] + altitude[3]) / 4;

    while (1)
    {
        ESP_ERROR_CHECK(hp203b_read_pressure(&pressure));
        ESP_LOGI(TAG, "Pressure: %f", pressure);
        ESP_ERROR_CHECK(hp203b_read_temperature(&temperature));
        ESP_LOGI(TAG, "Temperature: %f", temperature);
        ESP_ERROR_CHECK(hp203b_read_altitude(&tmp));
        if (tmp > (altitude[i % 4] - 0.5) && tmp < (altitude[i % 4] + 0.5))
        {
            altitude[i % 4] = tmp;
        }
        else
        {
            altitude[i % 4] = (3 * avg + tmp) / 4;
        }
        avg = (altitude[0] + altitude[1] + altitude[2] + altitude[3]) / 4;
        ESP_LOGI(TAG, "Altitude: %f", avg);

        i++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
