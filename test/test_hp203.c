#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"

#include "../main/hp203.h"

const static char *TAG = "HP203_EXAMPLE";

#define I2C_MASTER_SCL_IO           8      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           9      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0      /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

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
    i2c_master_init();
    hp203b_init();

    float pressure, temperature, altitude;


    while (1) {
        ESP_ERROR_CHECK(hp203b_read_pressure(&pressure));
        ESP_LOGI(TAG, "Pressure: %f", pressure);
        ESP_ERROR_CHECK(hp203b_read_temperature(&temperature));
        ESP_LOGI(TAG, "Temperature: %f", temperature);
        ESP_ERROR_CHECK(hp203b_read_altitude(&altitude));
        ESP_LOGI(TAG, "Altitude: %f", altitude);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

