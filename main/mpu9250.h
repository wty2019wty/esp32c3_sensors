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
#define MPU9250_I2C_ADDR_ALT            0x69    /* AD0 接高 */
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
#define AK8963_REG_ST1                  0x02    /* bit0=DRDY 数据就绪 */
#define AK8963_REG_HXL                  0x03    /* 0x03~0x08 磁力计数据（小端序，L 在前） */
#define AK8963_REG_ST2                  0x09    /* bit3=HOFL 溢出 */
#define AK8963_REG_CNTL1                0x0A    /* 工作模式 */
#define AK8963_REG_CNTL2                0x0B    /* bit0=SRST 软复位 */
#define AK8963_REG_ASTC                 0x0C    /* 自检 */
#define AK8963_REG_I2CDIS               0x0F    /* 禁用 I2C */
#define AK8963_REG_ASAX                 0x10    /* Fuse ROM 灵敏度调整起始 */

/* AK8963 CNTL1 值 */
#define AK8963_CNTL1_POWER_DOWN         0x00    /* 掉电模式 */
#define AK8963_CNTL1_FUSE_ROM           0x0F    /* Fuse ROM 访问模式（读出厂校准） */
#define AK8963_CNTL2_SRST               0x01    /* 软复位 */

/* AK8963 ST1/ST2 位定义 */
#define AK8963_ST1_DRDY                 0x01u   /* 数据就绪 */
#define AK8963_ST2_HOFL                 0x08u   /* 磁力计测量溢出 */

/* WHOAMI 期望值（参考数据手册 PS-MPU-9250A-01 Rev1.1）
 * 注意：0x71/0x73 才内置 AK8963 磁力计；0x70 是 MPU6500（无磁力计，
 * 市面上常见被抹丝印冒充 MPU9250 的翻新货）。 */
#define MPU9250_WHOAMI_MPU9250          0x71    /* 正品 MPU-9250，含 AK8963 */
#define MPU9250_WHOAMI_MPU9255          0x73    /* MPU-9255，含 AK8963 */
#define MPU9250_WHOAMI_MPU6500          0x70    /* MPU-6500，无磁力计 */
#define AK8963_WIA_ID                   0x48

/* 寄存器写入值 */
#define MPU9250_PWR_MGMT_1_RESET        0x80    /* 软复位 */
#define MPU9250_PWR_MGMT_1_WAKE_PLL     0x01    /* 唤醒，时钟源选 PLL(陀螺 X) */
#define MPU9250_USER_CTRL_I2C_MST_OFF   0x00    /* 关闭内部 I2C 主机 */
#define MPU9250_INT_PIN_CFG_BYPASS      0x02    /* 使能 I2C Bypass，暴露 AK8963 */
#define MPU9250_ACCEL_FS_SEL_4G         0x08    /* ±4g  (AFS_SEL=01) */
#define MPU9250_GYRO_FS_SEL_500         0x08    /* ±500dps (FS_SEL=01)，分辨率更高 */
#define MPU9250_SMPLRT_DIV_100HZ        0x09    /* 采样率 = 1000/(1+9) = 100Hz */

/* 磁力计连续测量模式 2（100Hz）+ 16 位输出。
 * 注意：AK8963 CNTL1 的 bit4 为输出位宽选择（1=16bit）。
 * 任务书中的 0x06 为 14 位模式（灵敏度 0.6 uT/LSB）；
 * 为与 0.15 uT/LSB 灵敏度匹配，此处使用 0x16。 */
#define AK8963_CNTL1_CONT_MODE2_16BIT   0x16

/* 灵敏度换算。
 * 磁场读数 = 原始 LSB * (0.15 uT/LSB) * (Fuse ROM 每轴调整值)。
 * 参考：AK8963 手册，(adj-128)/256 + 1 为出厂灵敏度修正系数。 */
#define MPU9250_ACCEL_LSB_PER_G         8192.0f     /* ±4g  */
#define MPU9250_GYRO_LSB_PER_DPS        65.5f       /* ±500dps */
#define MPU9250_MAG_UT_PER_LSB          0.15f       /* 16 位输出 */

/* 数据长度 */
#define MPU9250_AXIS_BYTES              6           /* 三轴各 2 字节 */
#define AK8963_BURST_BYTES              7           /* HXL..HZH + ST2（小端序） */

/* I2C 操作超时（毫秒） */
#define MPU9250_I2C_TIMEOUT_MS          100

/**
 * @brief MPU9250 设备句柄
 */
typedef struct {
    i2c_master_dev_handle_t dev;        /* 0x68：加速度计/陀螺仪 */
    i2c_master_dev_handle_t mag_dev;    /* 0x0C：AK8963 磁力计 */
    float mag_adj[3];                   /* Fuse ROM 每轴灵敏度调整值 */
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
