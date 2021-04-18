/*
 * BMP280 Sensor Driver
 */

#include "bmp280.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_SCL  5   /* D1 on the NodeMCU board */
#define I2C_MASTER_SDA  4   /* D2 on the NodeMCU board */

/* Static funtions */
static void bmp280_start_forced_meas(struct bmp280_device *bmp280);
static inline uint8_t bmp280_is_meas_in_progress(struct bmp280_device *bmp280);
static inline void bmp280_wait_for_meas(struct bmp280_device *bmp280);
static void bmp280_read_raw_values(struct bmp280_device *bmp280);
static void bmp280_convert_temperature_raw_values(struct bmp280_device *bmp280);

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
    uint8_t id, ret;

    /* Initialize I2C */
    i2c_master_init();
    bmp280_read_reg(BMP280_CHIP_ID_REG, &id, 1);
    if (id == BMP280_CHIP_ID)
        ESP_LOGI("BMP280", "Sensor Found!");
    else
        ESP_LOGW("BMP280", "Sensor Not Found.");

    /* Set oversampling config */
    bmp280_read_reg(BMP280_CTRL_MEAS_REG, &ret, 1);
    ret |= BMP280_TEMP_OS_MASK & (bmp280->temp_os << BMP280_TEMP_OS_POS);
    ret |= BMP280_PRES_OS_MASK & (bmp280->pres_os << BMP280_PRES_OS_POS);
    bmp280_write_reg(BMP280_CTRL_MEAS_REG, &ret, 1);

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

/* Initiate a measurement */
static void bmp280_start_forced_meas(struct bmp280_device *bmp280)
{
    uint8_t ret;

    bmp280_read_reg(BMP280_CTRL_MEAS_REG, &ret, 1);
    ret |= BMP280_MODE_MASK & BMP280_FORCE;
    
    bmp280_write_reg(BMP280_CTRL_MEAS_REG, &ret, 1);
    ESP_LOGI("BMP280", "Measurement intiated.");
}

/* Check if measurement is ongoing */
static inline uint8_t bmp280_is_meas_in_progress(struct bmp280_device *bmp280)
{
    uint8_t ret;
    
    bmp280_read_reg(BMP280_STATUS_REG, &ret, 1);
    ret = ret & BMP280_MEAS_STATUS_MASK;

    return ret;
}

/* wait for measurement to complete */
static inline void bmp280_wait_for_meas(struct bmp280_device *bmp280)
{
    while (bmp280_is_meas_in_progress(bmp280));
}

/* Get raw, uncompensated values */
static void bmp280_read_raw_values(struct bmp280_device *bmp280)
{
    uint8_t raw_values[6];

    /*
     * First 3 registers are pressure MSB to XLSB (0xF7 to 0xF9),
     * the next 3 are for temperature (0xFA to 0xFC)
     * 20 bit data
     */
    bmp280_read_reg(BMP280_RAW_VAL_REG_START, raw_values, 6);

    bmp280->pressure_raw = raw_values[0];
    bmp280->pressure_raw <<= 8;
    bmp280->pressure_raw |= raw_values[1];
    bmp280->pressure_raw <<= 4;
    bmp280->pressure_raw |= (raw_values[2]>>4);
    
    bmp280->temperature_raw = raw_values[3];
    bmp280->temperature_raw <<= 8;
    bmp280->temperature_raw |= raw_values[4];
    bmp280->temperature_raw <<=4;
    bmp280->temperature_raw |= (raw_values[5]>>4);

    ESP_LOGI("BMP280", "Raw Temperature: %d", bmp280->temperature_raw);
    ESP_LOGI("BMP280", "Raw Pressure: %d", bmp280->pressure_raw);
}

/* Convert raw value to temperature, formula given in BMP280 datasheet */
static void bmp280_convert_temperature_raw_values(struct bmp280_device *bmp280)
{
    int32_t var1, var2, comp_temp;

    /* Keeping formulas expanded here, to understand them better */
    var1 = 
    (
        (
            (
                (int32_t)(bmp280->temperature_raw >> 3) - ((int32_t)bmp280->trim_params.dig_T1<<1)
            )
        )
        
        *
        ((int32_t)bmp280->trim_params.dig_T2)
    )

    >> 11 ;

    var2 =
    (
        (
            (
                (
                    (int32_t)(bmp280->temperature_raw >> 4) - ((int32_t)bmp280->trim_params.dig_T1)
                )
                
                *
                
                (
                    (int32_t)(bmp280->temperature_raw >> 4) - ((int32_t)bmp280->trim_params.dig_T1)
                )
            )
            
            >> 12
        )
        
        *
        
        ((int32_t)bmp280->trim_params.dig_T3)
    )
    >> 14 ;
    

    bmp280->t_fine = var1 + var2;
    comp_temp  = (bmp280->t_fine * 5 + 128) / 256;

    bmp280->temperature_val = comp_temp;

    ESP_LOGI("BMP280", "Temperature: %d.%02d Deg C", bmp280->temperature_val/100, bmp280->temperature_val%100);
}

/* Convert raw value to pressure, formula given in BMP280 datasheet */
static void bmp280_convert_pressure_raw_values(struct bmp280_device *bmp280)
{
    int32_t var1, var2;
    uint32_t comp_press;

    var1 = ((int32_t)bmp280->t_fine >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)bmp280->trim_params.dig_P6;
    var2 = var2 + ((var1 * ((int32_t)bmp280->trim_params.dig_P5)) << 1);
    var2 = (var2 >> 2) +  (((int32_t)bmp280->trim_params.dig_P4) << 16);
    var1 = (((bmp280->trim_params.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)bmp280->trim_params.dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)bmp280->trim_params.dig_P1))>>15);

    /* Avoid divide by 0 */
    if (var1 == 0) {
        comp_press = 0;
        bmp280->pressure_val = comp_press;
        return;
    }

    comp_press = (((uint32_t)(((int32_t)1048576)-bmp280->pressure_raw)-(var2>>12)))*3125;

    /* Avoid overflow */
    if (comp_press < 0x80000000) {
        comp_press = (comp_press << 1) / ((uint32_t)var1);
    } else {
        comp_press = (comp_press / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)bmp280->trim_params.dig_P9) * ((int32_t)(((comp_press>>3) * (comp_press>>3))>>13)))>>12;
    var2 = (((int32_t)(comp_press>>2)) * ((int32_t)bmp280->trim_params.dig_P8))>>13;
    comp_press = (uint32_t)((int32_t)comp_press + ((var1 + var2 + bmp280->trim_params.dig_P7) >> 4));
    
    bmp280->pressure_val = comp_press;

    ESP_LOGI("BMP280", "Pressure: %d.%02d hPa", bmp280->pressure_val/100, bmp280->pressure_val%100);
}


/* Get a single reading and go back to sleep */
void bmp280_oneshot_read(struct bmp280_device *bmp280)
{
    bmp280_start_forced_meas(bmp280);
    bmp280_wait_for_meas(bmp280);
    bmp280_read_raw_values(bmp280);
    bmp280_convert_temperature_raw_values(bmp280);
    bmp280_convert_pressure_raw_values(bmp280);
}

/* Sensor Reading Task */
void bmp280_task(void *bmp280_inst)
{
    struct bmp280_device *bmp280;

    bmp280 = (struct bmp280_device *)bmp280_inst;

    bmp280_init(bmp280);

    while (1) {
        bmp280_oneshot_read(bmp280);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}