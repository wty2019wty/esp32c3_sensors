/*
 * I2C 总线统一配置（所有从设备共用同一时钟频率与超时）
 *
 * 注意：ESP-IDF v6.1 新版 i2c_master 驱动的时钟是在
 * i2c_device_config_t.scl_speed_hz 中按设备设置的，总线配置结构体里没有时钟字段。
 * 本文件位于 main/；app_main 初始化各驱动时传入 I2C_SCL_SPEED_HZ。
 * 引脚宏仅用于 main 中的总线初始化与线电平自检。
 */
#pragma once

#include "hal/gpio_types.h"

/* ---------------- 引脚定义 ----------------
 * ESP32-C3 Super Mini 默认：SDA=GPIO8, SCL=GPIO9。
 * 注意：GPIO8 在很多 Super Mini 板上接了板载 LED，可能拉低 SDA，
 * 若自检发现 SDA 无法拉高，可临时改到空闲引脚（如 GPIO6/GPIO7）验证。
 */
#define I2C_SDA_GPIO            GPIO_NUM_8
#define I2C_SCL_GPIO            GPIO_NUM_9

/* SCL 频率：
 *   400000  -> MPU-9250 数据手册规定最高 400kHz (Fast-mode)
 *   100000  -> 排查用
 */
#define I2C_SCL_SPEED_HZ        400000

/* 所有 I2C 操作统一超时（毫秒） */
#define I2C_BUS_TIMEOUT_MS      100
