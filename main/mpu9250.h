/*
 * MPU9250 九轴 IMU 驱动（加速度计/陀螺仪 0x68，AK8963 磁力计 0x0C）
 * 参考：InvenSense MPU-9250 Register Map Rev 1.4 / AK8963 Datasheet
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* I2C 从机地址 */
#define MPU9250_I2C_ADDR                0x68    /* AD0 接地 */
#define AK8963_I2C_ADDR                 0x0C    /* MPU9250 内置磁力计 */

/* MPU9250 寄存器（Register Map） */
#define MPU9250_REG_SMPLRT_DIV          0x19
#define MPU9250_REG_CONFIG              0x1A
#define MPU9250_REG_GYRO_CONFIG         0x1B
#define MPU9250_REG_ACCEL_CONFIG        0x1C
#define MPU9250_REG_INT_PIN_CFG         0x37    /* 含 I2C_BYPASS_EN */
#define MPU9250_REG_ACCEL_XOUT_H        0x3B    /* 0x3B~0x40 加速度 */
#define MPU9250_REG_GYRO_XOUT_H         0x43    /* 0x43~0x48 陀螺仪 */
#define MPU9250_REG_USER_CTRL           0x6A    /* 含 I2C_MST_EN */
#define MPU9250_REG_PWR_MGMT_1          0x6B
#define MPU9250_REG_WHO_AM_I            0x75

/* AK8963 寄存器 */
#define AK8963_REG_WIA                  0x00    /* 器件 ID，期望 0x48 */
#define AK8963_REG_ST1                  0x02    /* 数据就绪标志 */
#define AK8963_REG_HXL                  0x03    /* 0x03~0x08 磁力计数据 */
#define AK8963_REG_ST2                  0x09    /* 溢出标志 */
#define AK8963_REG_CNTL1                0x0A    /* 工作模式 */
#define AK8963_REG_ASAX                 0x10    /* 灵敏度调整起始 */

/* WHOAMI 期望值 */
#define MPU9250_WHOAMI_MPU9250          0x71
#define MPU9250_WHOAMI_MPU9255          0x73
#define AK8963_WIA_ID                   0x48

/* 寄存器写入值 */
#define MPU9250_PWR_MGMT_1_RESET        0x80    /* 软复位 */
#define MPU9250_PWR_MGMT_1_WAKE_PLL     0x01    /* 唤醒，时钟源选 PLL(陀螺 X) */
#define MPU9250_USER_CTRL_I2C_MST_OFF   0x00    /* 关闭内部 I2C 主机 */
#define MPU9250_INT_PIN_CFG_BYPASS      0x02    /* 使能 I2C Bypass，暴露 AK8963 */
#define MPU9250_ACCEL_FS_SEL_4G         0x08    /* ±4g  (AFS_SEL=01) */
#define MPU9250_GYRO_FS_SEL_2000        0x18    /* ±2000dps (FS_SEL=11) */
#define MPU9250_SMPLRT_DIV_100HZ        0x09    /* 采样率 = 1000/(1+9) = 100Hz */

/* 磁力计连续测量模式 2（100Hz）+ 16 位输出。
 * 注意：AK8963 CNTL1 的 bit4 为输出位宽选择（1=16bit）。
 * 任务书中的 0x06 为 14 位模式（灵敏度 0.6 uT/LSB）；
 * 为与 0.15 uT/LSB 灵敏度匹配，此处使用 0x16。 */
#define AK8963_CNTL1_CONT_MODE2_16BIT   0x16

/* 灵敏度换算 */
#define MPU9250_ACCEL_LSB_PER_G         8192.0f     /* ±4g  */
#define MPU9250_GYRO_LSB_PER_DPS        16.384f     /* ±2000dps */
#define MPU9250_MAG_UT_PER_LSB          0.15f       /* 16 位输出 */

/* 数据长度 */
#define MPU9250_AXIS_BYTES              6           /* 三轴各 2 字节 */
#define AK8963_BURST_BYTES              8           /* ST1 + 6 数据 + ST2 */

/* I2C 操作超时（毫秒） */
#define MPU9250_I2C_TIMEOUT_MS          100

/**
 * @brief MPU9250 设备句柄
 */
typedef struct {
    i2c_master_dev_handle_t dev;        /* 0x68：加速度计/陀螺仪 */
    i2c_master_dev_handle_t mag_dev;    /* 0x0C：AK8963 磁力计 */
    bool present;                       /* MPU9250 是否在线 */
    bool mag_present;                   /* AK8963 是否在线 */
} mpu9250_t;

/**
 * @brief 一次采样的九轴数据
 */
typedef struct {
    float acc_x;    /* g */
    float acc_y;
    float acc_z;
    float gyro_x;   /* °/s */
    float gyro_y;
    float gyro_z;
    float mag_x;    /* μT */
    float mag_y;
    float mag_z;
} mpu9250_sample_t;

/**
 * @brief 探测并初始化 MPU9250 与 AK8963
 *
 * @param[out] mpu 设备句柄
 * @param[in]  bus I2C 主总线句柄
 * @return ESP_OK 成功；其它为错误码
 */
esp_err_t mpu9250_init(mpu9250_t *mpu, i2c_master_bus_handle_t bus);

/**
 * @brief 读取加速度计、陀螺仪与磁力计
 *
 * 加速度计与陀螺仪必须成功；磁力计失败时磁力数据置 0，不影响整体返回。
 *
 * @param[in]  mpu    设备句柄
 * @param[out] sample 采样结果
 * @return ESP_OK 成功；其它为错误码
 */
esp_err_t mpu9250_read(mpu9250_t *mpu, mpu9250_sample_t *sample);
