/*
 * SSD1315 OLED 驱动（128x64，指令集兼容 SSD1306，I2C 地址 0x3C）
 * 参考：SSD1315 / SSD1306 Datasheet
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* I2C 从机地址（常见 0x3C，部分模块为 0x3D） */
#define SSD1315_I2C_ADDR        0x3C
#define SSD1315_I2C_ADDR_ALT    0x3D

/* 屏幕尺寸 */
#define SSD1315_WIDTH           128
#define SSD1315_HEIGHT          64
#define SSD1315_PAGES           (SSD1315_HEIGHT / 8)    /* 8 页 */
#define SSD1315_BUF_SIZE        (SSD1315_WIDTH * SSD1315_PAGES)  /* 1024 字节 */

/* 6x8 字体 */
#define SSD1315_CHAR_WIDTH      6
#define SSD1315_CHAR_HEIGHT     8
#define SSD1315_MAX_COLS        (SSD1315_WIDTH / SSD1315_CHAR_WIDTH)  /* 21 */

/* I2C 控制字节：Co=0, D/C#=0 命令；Co=0, D/C#=1 数据 */
#define SSD1315_CTRL_CMD        0x00
#define SSD1315_CTRL_DATA       0x40

/* I2C 操作超时（毫秒）：OLED 整帧写入较慢，参考工程使用 1000ms */
#define SSD1315_I2C_TIMEOUT_MS  1000

/**
 * @brief SSD1315 设备句柄（含 1024 字节帧缓冲区）
 *
 * 结构体较大，建议使用静态/全局变量，避免占用任务栈。
 */
typedef struct {
    i2c_master_dev_handle_t dev;
    bool present;
    uint8_t buf[SSD1315_BUF_SIZE];
} ssd1315_t;

/**
 * @brief 探测、初始化 OLED 并清屏
 *
 * @param[out] oled 设备句柄
 * @param[in]  bus  I2C 主总线句柄
 * @return ESP_OK 成功；其它为错误码
 */
esp_err_t ssd1315_init(ssd1315_t *oled, i2c_master_bus_handle_t bus);

/**
 * @brief 清空帧缓冲区（调用后需 flush 才生效）
 */
void ssd1315_clear(ssd1315_t *oled);

/**
 * @brief 在帧缓冲区绘制一个字符
 *
 * @param[in] oled 设备句柄
 * @param[in] page 页号（0~7，每页 8 像素高）
 * @param[in] col  列号（0~20，每字符 6 像素宽）
 * @param[in] c    ASCII 字符（超出 0x20~0x7F 显示为 '?'）
 */
void ssd1315_draw_char(ssd1315_t *oled, uint8_t page, uint8_t col, char c);

/**
 * @brief 在帧缓冲区绘制字符串（自动截断到 21 列）
 *
 * @param[in] oled 设备句柄
 * @param[in] page 页号（0~7）
 * @param[in] col  起始列号（0~20）
 * @param[in] str  C 字符串
 */
void ssd1315_draw_string(ssd1315_t *oled, uint8_t page, uint8_t col, const char *str);

/**
 * @brief 将整个帧缓冲区刷新到 OLED（按页写入）
 *
 * @param[in] oled 设备句柄
 * @return ESP_OK 成功；其它为 I2C 错误
 */
esp_err_t ssd1315_flush(ssd1315_t *oled);
