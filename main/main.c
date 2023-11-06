#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"

#include "hp203.h"
#include "mpu6050.h"

static const char *TAG = "I2C_EXAMPLE";
static mpu6050_handle_t mpu6050 = NULL;

#define I2C_MASTER_SCL_IO 8         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 9         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
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

/**
 * @brief i2c master initialization
 */
static void mpu6050_init(void)
{
    esp_err_t ret;

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);

    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS));

    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050));
}

void app_main(void)
{
    float tmp, pressure, temperature, altitude[4], avg;
    int i = 0;

    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    complimentary_angle_t complimentary_angle;

    i2c_master_init();
    hp203b_init();
    mpu6050_init();

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
        ESP_LOGI(TAG, "Altitude: %f\n", avg);

        i++;

        ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
        ESP_LOGI(TAG, "device id:%X", mpu6050_deviceid);

        ret = mpu6050_get_acce(mpu6050, &acce);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        ret = mpu6050_get_temp(mpu6050, &temp);
        ESP_LOGI(TAG, "t:%.2f", temp.temp);

        ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &complimentary_angle);
        ESP_LOGI(TAG, "angle_roll:%.2f, angle_pitch:%.2f", complimentary_angle.roll, complimentary_angle.pitch);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
    mpu6050_delete(mpu6050);
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
}
