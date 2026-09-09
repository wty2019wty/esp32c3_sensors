/*
 * BMP280 气压/温度传感器驱动（I2C 地址 0x76，GY-91 板载）
 * 参考：Bosch BMP280 Datasheet Rev 1.1
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* I2C 从机地址（SDO 接地时为 0x76） */
#define BMP280_I2C_ADDR             0x76

/* 寄存器地址（Datasheet Table 18） */
#define BMP280_REG_CHIP_ID          0xD0    /* WHOAMI */
#define BMP280_REG_RESET            0xE0
#define BMP280_REG_CALIB            0x88    /* 校准参数起始，共 26 字节到 0xA1 */
#define BMP280_REG_CTRL_MEAS        0xF4
#define BMP280_REG_CONFIG           0xF5
#define BMP280_REG_PRESS_MSB        0xF7    /* 0xF7~0xF9 气压，0xFA~0xFC 温度 */
#define BMP280_REG_DATA_LEN         6

/* WHOAMI 期望值 */
#define BMP280_CHIP_ID              0x58

/* 校准参数长度：0x88 ~ 0xA1 */
#define BMP280_CALIB_LEN            26

/* ctrl_meas：osrs_t=x2 (010), osrs_p=x16 (101), mode=normal (11) -> 0b0101_0111 */
#define BMP280_CTRL_MEAS_NORMAL     0x57

/* config：t_sb=0.5ms, filter=off, 3-wire SPI 关闭 -> 0x00 */
#define BMP280_CONFIG_DEFAULT       0x00

/* 海平面标准大气压（Pa） */
#define BMP280_SEA_LEVEL_PA         101325.0f

/* 海拔公式指数 1/5.255 ≈ 0.1903 */
#define BMP280_ALT_EXPONENT         0.1903f

/* 1 hPa = 100 Pa */
#define BMP280_PA_PER_HPA           100.0f

/* I2C 操作超时（毫秒） */
#define BMP280_I2C_TIMEOUT_MS       100

/**
 * @brief BMP280 校准参数与设备句柄
 */
typedef struct {
    uint16_t dig_T1;            /* 温度校准系数 */
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;            /* 气压校准系数 */
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    i2c_master_dev_handle_t dev;
    bool present;
} bmp280_t;

/**
 * @brief 在 I2C 总线上探测、初始化并读取校准参数
 *
 * @param[out] bmp 设备句柄
 * @param[in]  bus I2C 主总线句柄
 * @return ESP_OK 成功；其它为错误码
 */
esp_err_t bmp280_init(bmp280_t *bmp, i2c_master_bus_handle_t bus);

/**
 * @brief 读取温度、气压与海拔
 *
 * 内部先计算温度得到中间量 t_fine，再据此补偿气压（温度->气压耦合）。
 *
 * @param[in]  bmp       设备句柄
 * @param[out] temp_c    温度（摄氏度）
 * @param[out] press_hpa 气压（hPa）
 * @param[out] alt_m     海拔（米）
 * @param[out] t_fine    温度补偿中间变量（可为 NULL）
 * @return ESP_OK 成功；其它为错误码
 */
esp_err_t bmp280_read(bmp280_t *bmp, float *temp_c, float *press_hpa, float *alt_m, int32_t *t_fine);

/**
 * @brief 仅做温度补偿，得到 t_fine（供内部/调试使用）
 *
 * @param[in]  bmp    设备句柄
 * @param[in]  adc_t  原始温度 ADC 值
 * @param[out] temp_c 温度（摄氏度）
 * @param[out] t_fine 中间变量
 */
void bmp280_compensate_temperature(const bmp280_t *bmp, int32_t adc_t, float *temp_c, int32_t *t_fine);
