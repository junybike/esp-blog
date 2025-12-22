#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

/* I2C */

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    GPIO_NUM_21
#define I2C_SCL_GPIO    GPIO_NUM_22
#define I2C_FREQ_HZ     100000

#define BME280_ADDR     0x76
#define BME280_REG_ID   0xD0

static const char *TAG = "BME280";

/* Registers */

#define REG_ID          0xD0
#define REG_RESET       0xE0
#define REG_CTRL_HUM    0xF2
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_DATA        0xF7

/* Calibration */

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

static bme280_calib_t calib;
static int32_t t_fine;

/* I2C Helpers */

static esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_PORT,
        BME280_ADDR,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t i2c_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(
        I2C_PORT,
        BME280_ADDR,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(100)
    );
}

/* Calibration */

static void bme280_read_calibration(void)
{
    uint8_t buf1[26];
    uint8_t buf2[7];

    i2c_read(0x88, buf1, 26);
    i2c_read(0xE1, buf2, 7);

    calib.dig_T1 = (buf1[1] << 8) | buf1[0];
    calib.dig_T2 = (buf1[3] << 8) | buf1[2];
    calib.dig_T3 = (buf1[5] << 8) | buf1[4];

    calib.dig_P1 = (buf1[7] << 8) | buf1[6];
    calib.dig_P2 = (buf1[9] << 8) | buf1[8];
    calib.dig_P3 = (buf1[11] << 8) | buf1[10];
    calib.dig_P4 = (buf1[13] << 8) | buf1[12];
    calib.dig_P5 = (buf1[15] << 8) | buf1[14];
    calib.dig_P6 = (buf1[17] << 8) | buf1[16];
    calib.dig_P7 = (buf1[19] << 8) | buf1[18];
    calib.dig_P8 = (buf1[21] << 8) | buf1[20];
    calib.dig_P9 = (buf1[23] << 8) | buf1[22];

    calib.dig_H1 = buf1[25];
    calib.dig_H2 = (buf2[1] << 8) | buf2[0];
    calib.dig_H3 = buf2[2];
    calib.dig_H4 = (buf2[3] << 4) | (buf2[4] & 0x0F);
    calib.dig_H5 = (buf2[5] << 4) | (buf2[4] >> 4);
    calib.dig_H6 = (int8_t)buf2[6];
}

/* Compensation */

static int32_t compensate_temp(int32_t adc)
{
    int32_t var1 = ((((adc >> 3) - ((int32_t)calib.dig_T1 << 1))) * calib.dig_T2) >> 11;
    int32_t var2 = (((((adc >> 4) - calib.dig_T1) * ((adc >> 4) - calib.dig_T1)) >> 12) * calib.dig_T3) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

/* Task */

void bme280_task(void *arg)
{
    uint8_t data[8];

    while (1)
    {
        i2c_read(REG_DATA, data, 8);

        int32_t adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        int32_t adc_h = (data[6] << 8) | data[7];

        int32_t temp = compensate_temp(adc_t);

        ESP_LOGI(TAG, "Temp = %.2f C | Raw Hum = %ld | Raw Press = %ld", temp / 100.0, adc_h, adc_p);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* App main */
void app_main(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };

    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);

    uint8_t id;
    i2c_read(REG_ID, &id, 1);
    ESP_LOGI(TAG, "Chip ID = 0x%02X", id);

    // Sensor config
    i2c_write(REG_CTRL_HUM, 0x01);
    i2c_write(REG_CTRL_MEAS, 0x27);
    i2c_write(REG_CONFIG, 0x00);

    bme280_read_calibration();

    xTaskCreate(bme280_task, "bme280_task", 4096, NULL, 5, NULL);
}
