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

static const char *TAG = "TinyPlaneFC";
static mpu6050_handle_t mpu6050 = NULL;
static hp203_handle_t hp203 = NULL;
static sensor_data_buffer_t sensor_data_buffer;
static uint32_t motor_duty_l = 0;
static uint32_t motor_duty_r = 0;
static float pressure = 0.0;
static float temperature = 0.0;
static float altitude = 0.0;



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
}

static void update_hp203_task(void *pvParameters)
{
    
    while (1)
    {
        if (xSemaphoreTake(sensor_data_buffer.mutex, 2 / portTICK_PERIOD_MS) == pdTRUE)
        {
            
            
            
            xSemaphoreGive(sensor_data_buffer.mutex);

        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void sensor_data_buffer_init(void)
{
    sensor_data_buffer.mutex = xSemaphoreCreateMutex();
    sensor_data_buffer.sensor_data_read = 0;
    sensor_data_buffer.sensor_data_write = 0;
}


static void update_sensor_task(void *pvParameters)
{
    hp203_convert_channel_t convert_channel = HP203_CONVERT_PT;
    hp203_convert_osr_t convert_osr = HP203_CONVERT_OSR_256;

    // while (xSemaphoreTake(sensor_data_buffer.mutex, 0 / portTICK_PERIOD_MS) != pdTRUE);
    while (1)
    {
        if (xSemaphoreTake(sensor_data_buffer.mutex, 2 / portTICK_PERIOD_MS) == pdTRUE)
        {
            ESP_ERROR_CHECK(hp203_start_convert(hp203, convert_channel, convert_osr));
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

            ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050,
                                            &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce));
            // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f",
            //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_x,
            //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_y,
            //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.acce.acce_z);
            ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050,
                                            &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro));
            // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f",
            //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_x,
            //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_y,
            //          sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.gyro.gyro_z);
            
            ESP_ERROR_CHECK(mpu6050_get_temp(mpu6050,
                                            &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp));
            // ESP_LOGI(TAG, "temp:%.2f", sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].mpu6050_data.temp.temp);
            
            if (convert_channel == HP203_CONVERT_PT)
            {
                // vTaskDelay((HP203_CONVERT_TIME >> convert_osr) / portTICK_PERIOD_MS);
                while (!hp203_pa_ready(hp203) || !hp203_t_ready(hp203))
                {
                    ESP_LOGI(TAG, "waiting for hp203 convert...");
                }
                ESP_ERROR_CHECK(hp203_read_pt(hp203, &pressure, &temperature));
                ESP_ERROR_CHECK(hp203_read_altitude(hp203, &altitude));
                // ESP_LOGI(TAG, "Pressure: %.2f, Temperature: %.2f, Altitude: %.2f", pressure, temperature, altitude);
            }
            else
            {
                // vTaskDelay((HP203_CONVERT_TIME >> (convert_osr + 1)) / portTICK_PERIOD_MS);
                while (!hp203_t_ready(hp203))
                {
                    ESP_LOGI(TAG, "waiting for hp203 convert...");
                }
                ESP_ERROR_CHECK(hp203_read_temperature(hp203, &temperature));
                ESP_LOGI(TAG, "Temperature: %.2f", temperature);
            }
            sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.pressure = pressure;
            sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.temperature = temperature;
            sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_write % SENSOR_DATA_BUFFER_MAX].hp203_data.altitude = altitude;
            
            ESP_LOGI(TAG, "data_write: %d", (unsigned int)sensor_data_buffer.sensor_data_write);
            sensor_data_buffer.sensor_data_write++;
            xSemaphoreGive(sensor_data_buffer.mutex);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    uint32_t sensor_data_size_to_send = 0;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif

        while (1) {
            ESP_LOGI(TAG, "Waiting for data");
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);
#else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
#endif
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                // rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                
                if (xSemaphoreTake(sensor_data_buffer.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    motor_duty_l = *(uint32_t *)(&rx_buffer[0]);
                    motor_duty_r = *(uint32_t *)(&rx_buffer[4]);
                    ESP_LOGI(TAG, "PWM Duty L: %d, R: %d", (int)motor_duty_l, (int)motor_duty_r);
                    sensor_data_size_to_send = sensor_data_buffer.sensor_data_write - sensor_data_buffer.sensor_data_read;
                    if (sensor_data_size_to_send > SENSOR_DATA_BUFFER_MAX)
                    {
                        sensor_data_size_to_send = SENSOR_DATA_BUFFER_MAX;
                        sensor_data_buffer.sensor_data_read = sensor_data_buffer.sensor_data_write - SENSOR_DATA_BUFFER_MAX;
                    }
                    for (int i = 0; i < sensor_data_size_to_send; i++)
                    {
                        err = sendto(sock,
                                     &sensor_data_buffer.sensor_data[sensor_data_buffer.sensor_data_read % SENSOR_DATA_BUFFER_MAX],
                                     sizeof(sensor_data_buffer.sensor_data[0]),
                                     0,
                                     (struct sockaddr *)&source_addr,
                                     sizeof(source_addr));
                        if (err < 0)
                        {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                            break;
                        }
                        ESP_LOGI(TAG, "Sensor Data Read: %d, Sent %d bytes to %s", (unsigned int)sensor_data_buffer.sensor_data_read, err, addr_str);
                        sensor_data_buffer.sensor_data_read++;
                    }
                    xSemaphoreGive(sensor_data_buffer.mutex);
                    motor_set_duty(motor_duty_l, motor_duty_r);
                }
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
        // xTaskCreate(update_hp203_task, "update_hp203", 4096, (void*)AF_INET, 7, NULL);
        xTaskCreate(update_sensor_task, "update_sensor", 4096, (void*)AF_INET, 4, NULL);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    uint8_t mac[6] = {0x58, 0xc7, 0xac, 0x74, 0xee, 0xa0};

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
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_AP, mac));
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

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif
}
