/*
 * SHT40 温湿度传感器驱动（I2C 地址 0x44）
 * 参考：Sensirion SHT4x Datasheet
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* I2C 从机地址（SHT4x 默认 0x44，ADDR 引脚拉高时为 0x45） */
#define SHT40_I2C_ADDR              0x44
#define SHT40_I2C_ADDR_ALT          0x45

/* 命令：高精度测量（Datasheet Table 9，命令 0xFD） */
#define SHT40_CMD_MEASURE_HIGH_PREC 0xFD

/* 高精度测量最大转换时间约 8.3ms，这里取 10ms 以满足 ">=10ms" 要求 */
#define SHT40_MEASURE_DELAY_MS      10

/* CRC-8 参数（Datasheet：多项式 0x31，初值 0xFF） */
#define SHT40_CRC_POLY              0x31
#define SHT40_CRC_INIT              0xFF

/* 单次测量返回字节数：T_MSB T_LSB T_CRC H_MSB H_LSB H_CRC */
#define SHT40_RAW_BYTES             6

/* I2C 操作超时（毫秒） */
#define SHT40_I2C_TIMEOUT_MS        100

/**
 * @brief SHT40 设备句柄
 */
typedef struct {
    i2c_master_dev_handle_t dev;    /* I2C 设备句柄 */
    bool present;                   /* 设备是否在线 */
} sht40_t;

/**
 * @brief 在 I2C 总线上探测并初始化 SHT40
 *
 * @param[out] sht 设备句柄
 * @param[in]  bus I2C 主总线句柄
 * @return ESP_OK 成功；其它为错误码（设备缺失时 present 置 false）
 */
esp_err_t sht40_init(sht40_t *sht, i2c_master_bus_handle_t bus);

/**
 * @brief 触发一次高精度测量并读取温湿度
 *
 * @param[in]  sht      设备句柄
 * @param[out] temp_c   温度（摄氏度）
 * @param[out] humi_rh  相对湿度（%RH，已限幅到 0~100）
 * @return ESP_OK 成功；ESP_ERR_INVALID_CRC 表示 CRC 校验失败；其它为 I2C 错误
 */
esp_err_t sht40_read(sht40_t *sht, float *temp_c, float *humi_rh);

/**
 * @brief 计算 Sensirion CRC-8（多项式 0x31，初值 0xFF）
 *
 * @param[in] data 数据指针
 * @param[in] len  数据长度
 * @return 8 位 CRC
 */
uint8_t sht40_crc8(const uint8_t *data, size_t len);
