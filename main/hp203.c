#include "hp203.h"

/**************************************************************************
I2C General Definitions
**************************************************************************/

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

/**************************************************************************
    CONVERSION DELAY (in hp)
**************************************************************************/
#define HP203B_CONVERSIONDELAY (100)

/**************************************************************************
    COMMAND REGISTER
**************************************************************************/
// Command to soft-reset the device (clears internal storage)
#define HP203B_CMD_SOFT_RESET (0x06)
// Convert the Sensor Output to the Digital Values.
// Triggers sensor reading and data will be stored to sensors nvram
// This command must be logicall Or'ed with the OSR settings and the
// Channel below
#define HP203B_CMD_CONVERT (0x40)

// Command to read the nvram registers
#define HP203B_CMD_READ_PT (0x10) // Read Temperature and Pressure Values
#define HP203B_CMD_READ_AT (0x11) // Read Temperature and Altitude Values
#define HP203B_CMD_READ_P (0x30)  // Read Pressure Value Only
#define HP203B_CMD_READ_A (0x31)  // Read Altitude Value Only
#define HP203B_CMD_READ_T (0x32)  // Read Temperature Value Only

// Commands for callibration
#define HP203B_CMD_ANA_CAL (0x28)   // Re-Calibrate the Internal Analog Blocks
#define HP203B_CMD_READ_REG (0x80)  // Read Out the Control Registers
#define HP203B_CMD_WRITE_REG (0xC0) // Write Out the Control Registers

/* OSR Settings */
#define HP203B_CMD_OSR_4096 (0x00) // Convert D1 (OSR=4096)
#define HP203B_CMD_OSR_2048 (0x04) // OSR: 2048
#define HP203B_CMD_OSR_1024 (0x08) // OSR: 1024
#define HP203B_CMD_OSR_512 (0x0C)  // OSR: 512
#define HP203B_CMD_OSR_256 (0x10)  // OSR: 256
#define HP203B_CMD_OSR_128 (0x14)  // OSR: 128
/* Channel */
#define HP203B_CMD_CHNL_PRESTEMP (0x00) // Sensor Pressure and Temperature Channel
#define HP203B_CMD_CHNL_TEMP (0x10)     // Sensor Temperature Channel

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t hp203b_write_cmd(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t hp203b_start_convert(i2c_port_t i2c_num)
{
    esp_err_t ret;

    /* First write a start ADC convert command:
     *
     * 010 + 3 bit OSR + 2 bit channel
     *
     * For our test, we are set to OSR 2048, and the pressure and temp channel
     */
    uint8_t command = HP203B_CMD_CONVERT | HP203B_CMD_OSR_2048 | HP203B_CMD_CHNL_PRESTEMP;
    /* Write the command to I2C */
    ret = hp203b_write_cmd(i2c_num, HP203_I2C_ADDR, (uint8_t *)&command, 1);
    /* Conversion delay is based on the set OSR, for OSR of 2048, we
     * wait for approx 65.6ms based on specs */
    vTaskDelay(70 / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t hp203b_get_sensor_data(i2c_port_t i2c_num, uint8_t sensorcmd, float *value)
{
    esp_err_t err;
    uint8_t data[4];

    /* Trigger start sensor reading and conversion */
    hp203b_start_convert(i2c_num);

    /* Now read the pressure by sending the command for read pressure*/
    hp203b_write_cmd(i2c_num, HP203_I2C_ADDR, (uint8_t *)&sensorcmd, 1);

    /* Now read a 3 byte value from the NVram registers */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    size_t size = 3;
    i2c_master_write_byte(cmd, (HP203_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, ((uint8_t *)&data[0]), size - 1, I2C_MASTER_ACK);
    }
    /* Read the last byte with nack */
    i2c_master_read_byte(cmd, ((uint8_t *)&data[0]) + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Return the pressure data
    *value = (((data[0] & 0x0F) << 16) + (data[1] << 8) + data[2]) / 100.0;

    return err;
}

esp_err_t hp203b_init()
{
    /* Initialize sensor, do a soft reset command */
    i2c_port_t i2cnum = HP203_I2C_PORT;
    uint8_t command = HP203B_CMD_SOFT_RESET;
    return hp203b_write_cmd(i2cnum, HP203_I2C_ADDR, (uint8_t *)&command, 1);
}

esp_err_t hp203b_read_pressure(float *pressure)
{
    i2c_port_t i2cnum = HP203_I2C_PORT;
    return hp203b_get_sensor_data(i2cnum, HP203B_CMD_READ_P, pressure);
}

esp_err_t hp203b_read_temperature(float *temperature)
{
    i2c_port_t i2cnum = HP203_I2C_PORT;
    return hp203b_get_sensor_data(i2cnum, HP203B_CMD_READ_T, temperature);
}

esp_err_t hp203b_read_altitude(float *altitude)
{
    i2c_port_t i2cnum = HP203_I2C_PORT;
    return hp203b_get_sensor_data(i2cnum, HP203B_CMD_READ_A, altitude);
}
