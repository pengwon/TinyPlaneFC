#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"
#include "esp_log.h"

#include "bat.h"
#include "motor.h"
#include "mpu6050.h"
#include "hp203.h"


#define I2C_MASTER_SCL_IO 8         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 9         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define SENSOR_DATA_BUFFER_MAX 16

typedef struct {
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
} mpu6050_data_t;

typedef struct {
    float pressure;
    float temperature;
    float altitude;
} hp203_data_t;

typedef struct {
    bat_data_t bat_data;
    motor_data_t motor_data;
    mpu6050_data_t mpu6050_data;
    hp203_data_t hp203_data;
} sensor_data_t;

typedef struct {
    SemaphoreHandle_t mutex;
    uint32_t sensor_data_read;
    uint32_t sensor_data_write;
    sensor_data_t sensor_data[SENSOR_DATA_BUFFER_MAX];
} sensor_data_buffer_t;

adc_oneshot_unit_handle_t adc1_handle;

static const char *TAG = "I2C_EXAMPLE";
static mpu6050_handle_t mpu6050 = NULL;
static hp203_handle_t hp203 = NULL;
static sensor_data_buffer_t sensor_data_buffer;
static uint32_t motor_duty_l = 0;
static uint32_t motor_duty_r = 0;

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
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS));
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050));
}

static void read_mpu6050_task(void *pvParameters)
{
    esp_err_t ret;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    complimentary_angle_t complimentary_angle;

    while (1) {
        ret = mpu6050_get_acce(mpu6050, &acce);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        ret = mpu6050_get_temp(mpu6050, &temp);
        ESP_LOGI(TAG, "t:%.2f", temp.temp);

        ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &complimentary_angle);
        ESP_LOGI(TAG, "angle_roll:%.2f, angle_pitch:%.2f", complimentary_angle.roll, complimentary_angle.pitch);
    }
    mpu6050_delete(mpu6050);

}

static void hp203_init(void)
{
    hp203 = hp203_create(I2C_MASTER_NUM, HP203_I2C_ADDRESS_CSB_0);
    ESP_ERROR_CHECK(hp203_reset(hp203));
    while(!hp203_dev_ready(hp203)) {
        ESP_LOGI(TAG, "waiting for hp203 ready...");
        vTaskDelay(5 / portTICK_PERIOD_MS);
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

static void sensor_data_buffer_init(void)
{
    sensor_data_buffer.mutex = xSemaphoreCreateMutex();
    sensor_data_buffer.sensor_data_read = 0;
    sensor_data_buffer.sensor_data_write = 0;
}

static void update_sensor_data(void)
{
    hp203_convert_channel_t convert_channel = HP203_CONVERT_PT;
    hp203_convert_osr_t convert_osr = HP203_CONVERT_OSR_256;

    ESP_ERROR_CHECK(hp203_start_convert(hp203, convert_channel, convert_osr));
    if (xSemaphoreTake(sensor_data_buffer.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE)
    {
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.voltage = bat_get_voltage();
        ESP_LOGI(TAG, "BAT Voltage: %d", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.voltage);
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.current = bat_get_current();
        ESP_LOGI(TAG, "BAT Current: %d", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.current);
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.temperature = bat_get_temperature();
        ESP_LOGI(TAG, "BAT Temperature: %d", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.temperature);

        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_l = motor_duty_l;
        ESP_LOGI(TAG, "Motor Duty L: %lu", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_l);
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_r = motor_duty_r;
        ESP_LOGI(TAG, "Motor Duty R: %lu", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_r);
        motor_get_current(&sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_l,
                          &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_r);
        ESP_LOGI(TAG, "Motor Current L: %ld", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_l);

        ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050,
                                         &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce));
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f",
                 sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_x,
                 sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_y,
                 sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_z);
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050,
                                         &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro));
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f",
                 sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_x,
                 sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_y,
                 sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_z);
        
        ESP_ERROR_CHECK(mpu6050_get_temp(mpu6050,
                                         &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp));
        ESP_LOGI(TAG, "temp:%.2f", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp.temp);
        
        if (convert_channel == HP203_CONVERT_PT)
        {
            vTaskDelay((HP203_CONVERT_TIME >> convert_osr) / portTICK_PERIOD_MS);
            while (!hp203_pa_ready(hp203) || !hp203_t_ready(hp203))
            {
                ESP_LOGI(TAG, "waiting for hp203 convert...");
            }
            ESP_ERROR_CHECK(hp203_read_pt(hp203,
                                          &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.pressure,
                                          &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.temperature));
            ESP_ERROR_CHECK(hp203_read_altitude(hp203,
                                                &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.altitude));
            ESP_LOGI(TAG, "Pressure: %.2f, Temperature: %.2f, Altitude: %.2f",
                     sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.pressure,
                     sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.temperature,
                     sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.altitude);
        }
        else
        {
            vTaskDelay((HP203_CONVERT_TIME >> (convert_osr + 1)) / portTICK_PERIOD_MS);
            while (!hp203_t_ready(hp203))
            {
                ESP_LOGI(TAG, "waiting for hp203 convert...");
            }
            ESP_ERROR_CHECK(hp203_read_temperature(hp203,
                                                   &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.temperature));
            ESP_LOGI(TAG, "Temperature: %.2f", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.temperature);
        }
        ESP_LOGI(TAG, "data_write: %lu", sensor_data_buffer.sensor_data_write++);
        xSemaphoreGive(sensor_data_buffer.mutex);
    }
}

void app_main(void)
{
    uint8_t mpu6050_deviceid;

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------Motor ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = MOTOR_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, L_MOTOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, R_MOTOR_ADC_CHANNEL, &config));

    //-------------BAT ADC Config---------------//
    config.atten = BAT_VOLTAGE_ADC_ATTEN;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_VOLTAGE_ADC_CHANNEL, &config));

    config.atten = BAT_CURRENT_ADC_ATTEN;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_CURRENT_ADC_CHANNEL, &config));

    config.atten = BAT_TEMPERATURE_ADC_ATTEN;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_TEMPERATURE_ADC_CHANNEL, &config));

    motor_init();
    i2c_master_init();
    hp203_init();
    mpu6050_init();
    sensor_data_buffer_init();

    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid));
    ESP_LOGI(TAG, "device id:%X", mpu6050_deviceid);

    // xTaskCreate(read_hp203_task, "read_hp203", 4096, NULL, 10, NULL);
    // 创建定时器
    TimerHandle_t timer = xTimerCreate(
        "MyTimer",          // 定时器名称
        pdMS_TO_TICKS(1000), // 周期性定时器的周期，这里是 1000 毫秒
        pdTRUE,             // 定时器在一次触发后自动重置
        (void *)0,          // 定时器ID，可以用于传递参数
        update_sensor_data      // 定时器回调函数
    );

    // 启动定时器
    if (timer != NULL) {
        xTimerStart(timer, 0);
    }

    // 这里可以继续执行其他初始化操作或任务
    // ...

    // 进入 FreeRTOS 任务调度器
    // vTaskStartScheduler();

    // while (1)
    // {
        

        

        

    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }
    
    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
}
