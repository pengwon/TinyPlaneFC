#include "hp203.h"
#include "esp_log.h"


// Command to soft-reset the device (clears internal storage)
#define HP203_CMD_SOFT_RESET (0x06)

// Convert the Sensor Output to the Digital Values.
// Triggers sensor reading and data will be stored to sensors nvram
// This command must be logicall Or'ed with the OSR settings and the
// Channel below
#define HP203_CMD_CONVERT (0x40)

// Command to read the nvram registers
#define HP203_CMD_READ_PT (0x10) // Read Temperature and Pressure Values
#define HP203_CMD_READ_AT (0x11) // Read Temperature and Altitude Values
#define HP203_CMD_READ_P (0x30)  // Read Pressure Value Only
#define HP203_CMD_READ_A (0x31)  // Read Altitude Value Only
#define HP203_CMD_READ_T (0x32)  // Read Temperature Value Only

// Commands for callibration
#define HP203_CMD_ANA_CAL (0x28)   // Re-Calibrate the Internal Analog Blocks
#define HP203_CMD_READ_REG (0x80)  // Read Out the Control Registers
#define HP203_CMD_WRITE_REG (0xC0) // Write Out the Control Registers

#define HP203_REG_ALT_OFF_L     0x00
#define HP203_REG_ALT_OFF_H     0x01
#define HP203_REG_PA_H_TH_L     0x02
#define HP203_REG_PA_H_TH_H     0x03
#define HP203_REG_PA_M_TH_L     0x04
#define HP203_REG_PA_M_TH_H     0x05
#define HP203_REG_PA_L_TH_L     0x06
#define HP203_REG_PA_L_TH_H     0x07
#define HP203_REG_T_H_TH        0x08
#define HP203_REG_T_M_TH        0x09
#define HP203_REG_T_L_TH        0x0A
#define HP203_REG_INT_EN        0x0B
#define HP203_REG_INT_CFG       0x0C
#define HP203_REG_INT_SRC       0x0D
#define HP203_REG_INT_DIR       0x0E
#define HP203_REG_PARA          0x0F

#define HP203_INT_SRC_DEV_RDY   (uint8_t) BIT6
#define HP203_INT_SRC_PA_RDY    (uint8_t) BIT5
#define HP203_INT_SRC_T_RDY     (uint8_t) BIT4


typedef struct {
    i2c_port_t bus;
    uint8_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} hp203_dev_t;

static esp_err_t hp203_read_reg(hp203_handle_t sensor, uint8_t reg, uint8_t *data)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    uint8_t command = HP203_CMD_READ_REG | reg;

    return i2c_master_write_read_device(sens->bus, sens->dev_addr, &command, 1, data, 1, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
}

static esp_err_t hp203_write_reg(hp203_handle_t sensor, uint8_t reg, uint8_t data)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    uint8_t write_data[2] = {HP203_CMD_WRITE_REG | reg, data};

    return i2c_master_write_to_device(sens->bus, sens->dev_addr, write_data, 2, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
}

hp203_handle_t hp203_create(i2c_port_t port, const uint8_t dev_addr)
{
    hp203_dev_t *sensor = (hp203_dev_t *) calloc(1, sizeof(hp203_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (hp203_handle_t) sensor;
}

void hp203_delete(hp203_handle_t sensor)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    free(sens);
}

esp_err_t hp203_reset(hp203_handle_t sensor)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    uint8_t command = HP203_CMD_SOFT_RESET;

    return i2c_master_write_to_device(sens->bus, sens->dev_addr, &command, 1, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
}

esp_err_t hp203_start_convert(hp203_handle_t sensor, hp203_convert_channel_t convert_channel, hp203_convert_osr_t convert_osr)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    uint8_t command = (0b010 << 5) | (convert_osr << 2) | convert_channel;
    
    return i2c_master_write_to_device(sens->bus, sens->dev_addr, &command, 1, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
}

esp_err_t hp203_read_pressure(hp203_handle_t sensor, float *pressure)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    esp_err_t ret;
    uint8_t data[3];
    uint8_t command = HP203_CMD_READ_P;

    ret = i2c_master_write_read_device(sens->bus, sens->dev_addr, &command, 1, data, 3, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Return the pressure data
    *pressure = (((data[0] & 0x0F) << 16) | (data[1] << 8) | data[2]) / 100.0;

    return ret;
}

esp_err_t hp203_read_temperature(hp203_handle_t sensor, float *temperature)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    esp_err_t ret;
    uint8_t data[3];
    uint8_t command = HP203_CMD_READ_T;

    ret = i2c_master_write_read_device(sens->bus, sens->dev_addr, &command, 1, data, 3, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Return the pressure data
    *temperature = (((data[0] & 0x0F) << 16) | (data[1] << 8) | data[2]) / 100.0;

    return ret;
}

esp_err_t hp203_read_altitude(hp203_handle_t sensor, float *altitude)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    esp_err_t ret;
    uint8_t data[3];
    uint8_t command = HP203_CMD_READ_A;

    ret = i2c_master_write_read_device(sens->bus, sens->dev_addr, &command, 1, data, 3, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Return the pressure data
    *altitude = (((data[1] & 0x0F) << 8) | data[2]) / 100.0;

    return ret;
}

esp_err_t hp203_read_pt(hp203_handle_t sensor, float *pressure, float *temperature)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    esp_err_t ret;
    uint8_t data[6];
    uint8_t command = HP203_CMD_READ_PT;

    ret = i2c_master_write_read_device(sens->bus, sens->dev_addr, &command, 1, data, 6, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *temperature = (((data[0] & 0x0F) << 16) | (data[1] << 8) | data[2]) / 100.0;
    *pressure = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) / 100.0;

    return ret;
}

esp_err_t hp203_read_at(hp203_handle_t sensor, float *altitude, float *temperature)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;
    esp_err_t ret;
    uint8_t data[6];
    uint8_t command = HP203_CMD_READ_AT;

    ret = i2c_master_write_read_device(sens->bus, sens->dev_addr, &command, 1, data, 6, HP203_I2C_TIME_OUT / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *temperature = (((data[0] & 0x0F) << 16) | (data[1] << 8) | data[2]) / 100.0;
    *altitude = (((data[4] & 0x0F) << 8) | data[5]) / 100.0;

    return ret;
}

bool hp203_dev_ready(hp203_handle_t sensor)
{
    esp_err_t ret;
    uint8_t data;

    ret = hp203_read_reg(sensor, HP203_REG_INT_SRC, &data);
    if (ret != ESP_OK || (data & HP203_INT_SRC_DEV_RDY) == 0)
    {   
        return false;
    }

    return true;
}

bool hp203_pa_ready(hp203_handle_t sensor)
{
    esp_err_t ret;
    uint8_t data;

    ret = hp203_read_reg(sensor, HP203_REG_INT_SRC, &data);
    if (ret != ESP_OK || (data & HP203_INT_SRC_PA_RDY) == 0)
    {   
        return false;
    }

    return true;
}

bool hp203_t_ready(hp203_handle_t sensor)
{
    esp_err_t ret;
    uint8_t data;

    ret = hp203_read_reg(sensor, HP203_REG_INT_SRC, &data);
    if (ret != ESP_OK || (data & HP203_INT_SRC_T_RDY) == 0)
    {   
        return false;
    }
    // hp203_read_reg(sensor, 0x0F, &data);
    // ESP_LOGI("HP203", "ALT_OFF: %d\n", data);

    return true;
}

esp_err_t hp203_int_enable(hp203_handle_t sensor, uint8_t int_en)
{
    hp203_dev_t *sens = (hp203_dev_t *) sensor;

    return hp203_write_reg(sens, HP203_REG_INT_EN, int_en);
}
