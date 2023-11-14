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

static void hp203_init(void)
{
    hp203 = hp203_create(I2C_MASTER_NUM, HP203_I2C_ADDRESS_CSB_0);
    ESP_ERROR_CHECK(hp203_reset(hp203));
    while(!hp203_dev_ready(hp203)) {
        ESP_LOGI(TAG, "waiting for hp203 ready...");
    }
    ESP_ERROR_CHECK(hp203_int_enable(hp203, HP203_INT_PA_RDY_EN | HP203_INT_T_RDY_EN));
    ESP_LOGI(TAG, "hp203 ready!");
}


static void read_hp203_task(void *pvParameters)
{
    hp203_convert_channel_t convert_channel = HP203_CONVERT_PT;
    hp203_convert_osr_t convert_osr = HP203_CONVERT_OSR_256;
    float pressure, temperature, altitude;

    while (1)
    {
        ESP_ERROR_CHECK(hp203_start_convert(hp203, convert_channel, convert_osr));

        if (convert_channel == HP203_CONVERT_PT)
        {
            vTaskDelay((HP203_CONVERT_TIME >> convert_osr) / portTICK_PERIOD_MS);
            while (!hp203_pa_ready(hp203) || !hp203_t_ready(hp203))
            {
                ESP_LOGI(TAG, "waiting for hp203 convert...");
            }
            ESP_ERROR_CHECK(hp203_read_pt(hp203, &pressure, &temperature));
            ESP_ERROR_CHECK(hp203_read_altitude(hp203, &altitude));
            ESP_LOGI(TAG, "Pressure: %.2f, Temperature: %.2f, Altitude: %.2f", pressure, temperature, altitude);
        }
        else
        {
            vTaskDelay((HP203_CONVERT_TIME >> (convert_osr + 1)) / portTICK_PERIOD_MS);
            while (!hp203_t_ready(hp203))
            {
                ESP_LOGI(TAG, "waiting for hp203 convert...");
            }
            ESP_ERROR_CHECK(hp203_read_temperature(hp203, &temperature));
            ESP_LOGI(TAG, "Temperature: %f", temperature);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    hp203_delete(hp203);
}


void app_main(void)
{
    i2c_master_init();
    hp203b_init();

    float pressure, temperature, altitude;

    xTaskCreate(read_hp203_task, "read_hp203", 4096, NULL, 10, NULL);
    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

