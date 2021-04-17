/*
 * BMP280 Sensor Driver
 */

#include "bmp280.h"

#include <stdio.h>

#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_SCL  5   /* D1 on the NodeMCU board */
#define I2C_MASTER_SDA  4   /* D2 on the NodeMCU board */

/* Initialize I2C Master */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = 0;
    conf.scl_pullup_en = 0;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}


/* Write data to the BMP280 */
static esp_err_t bmp280_write_reg(uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((BMP280_I2C_ADDR_PRIM << 1) | (I2C_MASTER_WRITE)), true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_write(cmd, data, data_len, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/* Read data from BMP280 */
static esp_err_t bmp280_read_reg(uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((BMP280_I2C_ADDR_PRIM << 1) | (I2C_MASTER_WRITE)), true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((BMP280_I2C_ADDR_PRIM << 1) | (I2C_MASTER_READ)), true);
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void bmp280_init(struct bmp280_device *bmp280)
{
    uint8_t id;
    /* Initialize I2C */
    i2c_master_init();
    bmp280_read_reg(BMP280_CHIP_ID_REG, &id, 1);
    if (id == BMP280_CHIP_ID)
        ESP_LOGI("BMP280", "Sensor Found!");
    else
        ESP_LOGW("BMP280", "Sensor Not Found.");

    /* Read BMP280 Trimming/Calibration Parameters */
    bmp280_read_trim_params(bmp280);
}

void bmp280_read_trim_params(struct bmp280_device *bmp280)
{
    /* Note: BMP280_DIG_xx is the LSB address */

    uint8_t temp[2];

    /* Read temperature compensation parameters */
    bmp280_read_reg(BMP280_DIG_T1, temp, 2);
    bmp280->trim_params.dig_T1 = ((temp[1] << 8) | temp[0]);

    bmp280_read_reg(BMP280_DIG_T2, temp, 2);
    bmp280->trim_params.dig_T2 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_T2 = (int16_t)bmp280->trim_params.dig_T2;

    bmp280_read_reg(BMP280_DIG_T3, temp, 2);
    bmp280->trim_params.dig_T3 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_T3 = (int16_t)bmp280->trim_params.dig_T3;

    /* Read pressure compensation paramters */
    bmp280_read_reg(BMP280_DIG_P1, temp, 2);
    bmp280->trim_params.dig_P1 = ((temp[1] << 8) | temp[0]);

    bmp280_read_reg(BMP280_DIG_P2, temp, 2);
    bmp280->trim_params.dig_P2 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P2 = (int16_t)bmp280->trim_params.dig_P2;

    bmp280_read_reg(BMP280_DIG_P3, temp, 2);
    bmp280->trim_params.dig_P3 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P3 = (int16_t)bmp280->trim_params.dig_P3;

    bmp280_read_reg(BMP280_DIG_P4, temp, 2);
    bmp280->trim_params.dig_P4 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P4 = (int16_t)bmp280->trim_params.dig_P4;

    bmp280_read_reg(BMP280_DIG_P5, temp, 2);
    bmp280->trim_params.dig_P5 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P5 = (int16_t)bmp280->trim_params.dig_P5;

    bmp280_read_reg(BMP280_DIG_P6, temp, 2);
    bmp280->trim_params.dig_P6 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P6 = (int16_t)bmp280->trim_params.dig_P6;

    bmp280_read_reg(BMP280_DIG_P7, temp, 2);
    bmp280->trim_params.dig_P7 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P7 = (int16_t)bmp280->trim_params.dig_P7;

    bmp280_read_reg(BMP280_DIG_P8, temp, 2);
    bmp280->trim_params.dig_P8 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P8 = (int16_t)bmp280->trim_params.dig_P8;

    bmp280_read_reg(BMP280_DIG_P9, temp, 2);
    bmp280->trim_params.dig_P9 = ((temp[1] << 8) | temp[0]);
    bmp280->trim_params.dig_P9 = (int16_t)bmp280->trim_params.dig_P9;

    /* Log to the console */
    ESP_LOGI("BMP280", "dig_T1: %d\t%x", bmp280->trim_params.dig_T1, bmp280->trim_params.dig_T1);
    ESP_LOGI("BMP280", "dig_T2: %d\t%x", bmp280->trim_params.dig_T2, bmp280->trim_params.dig_T2);
    ESP_LOGI("BMP280", "dig_T3: %d\t%x", bmp280->trim_params.dig_T3, bmp280->trim_params.dig_T3);
    ESP_LOGI("BMP280", "dig_P1: %d\t%x", bmp280->trim_params.dig_P1, bmp280->trim_params.dig_P1);
    ESP_LOGI("BMP280", "dig_P2: %d\t%x", bmp280->trim_params.dig_P2, bmp280->trim_params.dig_P2);
    ESP_LOGI("BMP280", "dig_P3: %d\t%x", bmp280->trim_params.dig_P3, bmp280->trim_params.dig_P3);
    ESP_LOGI("BMP280", "dig_P4: %d\t%x", bmp280->trim_params.dig_P4, bmp280->trim_params.dig_P4);
    ESP_LOGI("BMP280", "dig_P5: %d\t%x", bmp280->trim_params.dig_P5, bmp280->trim_params.dig_P5);
    ESP_LOGI("BMP280", "dig_P6: %d\t%x", bmp280->trim_params.dig_P6, bmp280->trim_params.dig_P6);
    ESP_LOGI("BMP280", "dig_P7: %d\t%x", bmp280->trim_params.dig_P7, bmp280->trim_params.dig_P7);
    ESP_LOGI("BMP280", "dig_P8: %d\t%x", bmp280->trim_params.dig_P8, bmp280->trim_params.dig_P8);
    ESP_LOGI("BMP280", "dig_P9: %d\t%x", bmp280->trim_params.dig_P9, bmp280->trim_params.dig_P9);
}