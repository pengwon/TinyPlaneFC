#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sys/param.h"
#include "soc/soc_caps.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "bat.h"
#include "motor.h"
#include "mpu6050.h"
#include "hp203.h"

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define ESP_WIFI_SSID      "TinyPlaneFC"
#define ESP_WIFI_PASS      "FC12345678"
#define ESP_WIFI_CHANNEL   6
#define MAX_STA_CONN       1

#define I2C_MASTER_SCL_IO 8         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 9         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define SENSOR_DATA_BUFFER_MAX 16

#define PORT CONFIG_EXAMPLE_PORT
#undef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT

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
    uint32_t index;
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

static const char *TAG = "TinyPlaneFC";
static mpu6050_handle_t mpu6050 = NULL;
static hp203_handle_t hp203 = NULL;
static sensor_data_buffer_t sensor_data_buffer;
static uint32_t motor_duty_l = 0;
static uint32_t motor_duty_r = 0;

char addr_str[128];
int addr_family = AF_INET;
int ip_protocol = 0;
int sock = 0;
bool wifi_connected = false;
struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
socklen_t socklen = sizeof(source_addr);
struct sockaddr_in dest_addr;

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


static void hp203_init(void)
{
    hp203 = hp203_create(I2C_MASTER_NUM, HP203_I2C_ADDRESS_CSB_0);
    ESP_ERROR_CHECK(hp203_reset(hp203));
    while(!hp203_dev_ready(hp203)) {
        ESP_LOGI(TAG, "waiting for hp203 ready...");
    }
    ESP_ERROR_CHECK(hp203_int_enable(hp203, HP203_INT_PA_RDY_EN | HP203_INT_T_RDY_EN));
    ESP_LOGI(TAG, "hp203 ready!");
    ESP_ERROR_CHECK(hp203_start_convert(hp203, HP203_CONVERT_PT, HP203_CONVERT_OSR_256));
}

static void sensor_data_buffer_init(void)
{
    sensor_data_buffer.mutex = xSemaphoreCreateMutex();
    sensor_data_buffer.sensor_data_read = 0;
    sensor_data_buffer.sensor_data_write = 0;
}


static void update_sensor_task(void *pvParameters)
{
    float pressure = 0.0;
    float temperature = 0.0;
    float altitude = 0.0;

    while (xSemaphoreTake(sensor_data_buffer.mutex, portMAX_DELAY) == pdTRUE)
    {
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].index = sensor_data_buffer.sensor_data_write;

        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.voltage = bat_get_voltage();
        // ESP_LOGI(TAG, "BAT Voltage: %d", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.voltage);
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.current = bat_get_current();
        // ESP_LOGI(TAG, "BAT Current: %d", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.current);
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.temperature = bat_get_temperature();
        // ESP_LOGI(TAG, "BAT Temperature: %d", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].bat_data.temperature);

        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_l = motor_duty_l;
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_r = motor_duty_r;
        // ESP_LOGI(TAG, "Motor Duty L: %lu, R: %lu",
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_l,
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.duty_r);
        motor_get_current(&sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_l,
                        &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_r);
        // ESP_LOGI(TAG, "Motor Current L: %ld, R: %ld",
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_l, 
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].motor_data.current_r);
        
        // ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050,
        //                                 &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce));
        mpu6050_get_acce(mpu6050, &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce);
        // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f",
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_x,
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_y,
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_z);
        // ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050,
        //                                 &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro));
        mpu6050_get_gyro(mpu6050, &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro);
        // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f",
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_x,
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_y,
        //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_z);
        // ESP_ERROR_CHECK(mpu6050_get_temp(mpu6050,
        //                                 &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp));
        mpu6050_get_temp(mpu6050, &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp);
        // ESP_LOGI(TAG, "temp:%.2f", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp.temp);

        if (hp203_pa_ready(hp203) && hp203_t_ready(hp203))
        {
            ESP_ERROR_CHECK(hp203_read_pt(hp203, &pressure, &temperature));
            ESP_ERROR_CHECK(hp203_read_altitude(hp203, &altitude));
            ESP_ERROR_CHECK(hp203_start_convert(hp203, HP203_CONVERT_PT, HP203_CONVERT_OSR_256));
            // ESP_LOGI(TAG, "Pressure: %.2f, Temperature: %.2f, Altitude: %.2f", pressure, temperature, altitude);
        }
        
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.pressure = pressure;
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.temperature = temperature;
        sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.altitude = altitude;
        
        ESP_LOGI(TAG, "data_write: %d", (unsigned int)sensor_data_buffer.sensor_data_write);
        sensor_data_buffer.sensor_data_write++;
        xSemaphoreGive(sensor_data_buffer.mutex);

        if (!wifi_connected)
        {
            ESP_LOGI(TAG, "wifi disconnected, update_sensor_task exit...");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

static void udp_receive_task(void *pvParameters)
{
    char rx_buffer[128];

    while (1)
    {
        ESP_LOGI(TAG, "Waiting for data");
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        // Error occurred during receiving
        if (len < 0)
        {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        }
        // Data received
        else
        {
            // Get the sender's ip address as string
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

            // rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
            motor_duty_l = *(uint32_t *)(&rx_buffer[0]);
            motor_duty_r = *(uint32_t *)(&rx_buffer[4]);
            motor_set_duty(motor_duty_l, motor_duty_r);
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            ESP_LOGI(TAG, "PWM Duty L: %d, R: %d", (int)motor_duty_l, (int)motor_duty_r);
        }

        if (!wifi_connected)
        {
            ESP_LOGI(TAG, "wifi disconnected, udp_receive_task exit...");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

static void udp_send_task(void *pvParameters)
{
    uint32_t sensor_data_size_to_send = 0;

    while (xSemaphoreTake(sensor_data_buffer.mutex, portMAX_DELAY) == pdTRUE)
    {
        sensor_data_size_to_send = sensor_data_buffer.sensor_data_write - sensor_data_buffer.sensor_data_read;
        if (sensor_data_size_to_send > SENSOR_DATA_BUFFER_MAX)
        {
            sensor_data_size_to_send = SENSOR_DATA_BUFFER_MAX;
            sensor_data_buffer.sensor_data_read = sensor_data_buffer.sensor_data_write - SENSOR_DATA_BUFFER_MAX;
        }

        for (int i = 0; i < sensor_data_size_to_send; i++)
        {
            int err = sendto(sock,
                                &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_read % SENSOR_DATA_BUFFER_MAX],
                                sizeof(sensor_data_buffer.sensor_data[0]),
                                0,
                                (struct sockaddr *)&source_addr,
                                sizeof(source_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            }
            ESP_LOGI(TAG, "Sensor Data Read: %d, Sent %d bytes to %s", (unsigned int)sensor_data_buffer.sensor_data_read, err, addr_str);
            sensor_data_buffer.sensor_data_read++;
        }
        xSemaphoreGive(sensor_data_buffer.mutex);

        if (!wifi_connected)
        {
            ESP_LOGI(TAG, "wifi disconnected, udp_send_task exit...");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

static void udp_init(void)
{
    char rx_buffer[128];
    struct sockaddr_in *dest_addr_ip4 = &dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    ESP_LOGI(TAG, "Waiting for data");
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
    // Error occurred during receiving
    if (len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    }
    // Data received
    else {
        // Get the sender's ip address as string
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

        ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
    }
    xTaskCreate(udp_receive_task, "udp_receive", 1024 * 4, (void*)AF_INET, 4, NULL);
    xTaskCreate(udp_send_task, "udp_send", 1024 * 4, (void*)AF_INET, 4, NULL);
}

static void udp_deinit(void)
{
    ESP_LOGI(TAG, "Shutting down socket and waiting restart...");
    shutdown(sock, 0);
    close(sock);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        wifi_connected = true;
        udp_init();
        xTaskCreate(update_sensor_task, "updata_sensor", 1024 * 8, NULL, 5, NULL);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
        wifi_connected = false;
        udp_deinit();
    }
}

void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
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
    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid));
    ESP_LOGI(TAG, "device id:%X", mpu6050_deviceid);
    
    sensor_data_buffer_init();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // ESP_ERROR_CHECK(example_connect());

    // 这里可以继续执行其他初始化操作或任务
    // ...
}
