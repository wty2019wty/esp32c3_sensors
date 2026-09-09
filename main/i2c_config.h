/*
 * I2C 总线统一配置（所有从设备共用同一时钟频率与超时）
 *
 * 注意：ESP-IDF v6.1 新版 i2c_master 驱动的时钟是在
 * i2c_device_config_t.scl_speed_hz 中按设备设置的，总线配置结构体里没有时钟字段。
 * 因此所有驱动必须引用本文件的宏，才能统一改速。
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
 *   10000   -> 临时诊断：给弱上拉留足上升时间，验证 OLED 是否只是上拉不足
 *   100000  -> 诊断用，容错性更好
 *   400000  -> 任务书要求，需配合 4.7kΩ 外部上拉 + 短线
 */
#define I2C_SCL_SPEED_HZ        10000

/* 所有 I2C 操作统一超时（毫秒） */
#define I2C_BUS_TIMEOUT_MS      100
