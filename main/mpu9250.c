/*
 * MPU9250 九轴 IMU 驱动实现（ESP-IDF v6.1 新版 I2C master API）
 *
 * 三个子传感器分开读取：加速度计、陀螺仪（均在 0x68），
 * 磁力计 AK8963（0x0C，需先打开 MPU9250 的 I2C Bypass）。
 */
#include "mpu9250.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_config.h"

static const char *TAG = "mpu9250";

/* ---------- MPU9250（0x68）读写辅助 ---------- */

static esp_err_t mpu_read_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(mpu->dev, &reg, 1, buf, len, MPU9250_I2C_TIMEOUT_MS);
}

static esp_err_t mpu_write_reg(mpu9250_t *mpu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mpu->dev, buf, sizeof(buf), MPU9250_I2C_TIMEOUT_MS);
}

/* ---------- AK8963（0x0C）读写辅助 ---------- */

static esp_err_t mag_read_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(mpu->mag_dev, &reg, 1, buf, len, MPU9250_I2C_TIMEOUT_MS);
}

static esp_err_t mag_write_reg(mpu9250_t *mpu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mpu->mag_dev, buf, sizeof(buf), MPU9250_I2C_TIMEOUT_MS);
}

/* ---------- 初始化 ---------- */

static esp_err_t mpu9250_init_mag(mpu9250_t *mpu, i2c_master_bus_handle_t bus)
{
    mpu->mag_dev = NULL;
    mpu->mag_present = false;

    esp_err_t err = i2c_master_probe(bus, AK8963_I2C_ADDR, MPU9250_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 (0x%02X) 探测失败: %s", AK8963_I2C_ADDR, esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AK8963_I2C_ADDR,
        .scl_speed_hz = I2C_SCL_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &mpu->mag_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AK8963 添加设备失败: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t wia = 0;
    err = mag_read_regs(mpu, AK8963_REG_WIA, &wia, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "读取 AK8963 WIA 失败: %s", esp_err_to_name(err));
        return err;
    }
    if (wia != AK8963_WIA_ID) {
        ESP_LOGW(TAG, "AK8963 WIA 不匹配: 0x%02X (期望 0x%02X)", wia, AK8963_WIA_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 软复位后进入连续测量模式 2（100Hz，16 位输出） */
    (void)mag_write_reg(mpu, AK8963_REG_CNTL1, 0x00);   /* 先进入 power-down */
    vTaskDelay(pdMS_TO_TICKS(10));
    err = mag_write_reg(mpu, AK8963_REG_CNTL1, AK8963_CNTL1_CONT_MODE2_16BIT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 配置连续测量失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    mpu->mag_present = true;
    ESP_LOGI(TAG, "AK8963 初始化成功 (0x%02X), WIA=0x%02X", AK8963_I2C_ADDR, wia);
    return ESP_OK;
}

esp_err_t mpu9250_init(mpu9250_t *mpu, i2c_master_bus_handle_t bus)
{
    if (mpu == NULL || bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(mpu, 0, sizeof(*mpu));
    mpu->present = false;
    mpu->mag_present = false;

    /* 依次尝试主地址与备用地址 */
    static const uint8_t addrs[] = { MPU9250_I2C_ADDR, MPU9250_I2C_ADDR_ALT };
    uint8_t used_addr = 0;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        if (i2c_master_probe(bus, addrs[i], MPU9250_I2C_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = I2C_SCL_SPEED_HZ,
        };
        err = i2c_master_bus_add_device(bus, &dev_cfg, &mpu->dev);
        if (err == ESP_OK) {
            used_addr = addrs[i];
            break;
        }
    }
    if (used_addr == 0) {
        ESP_LOGE(TAG, "MPU9250 未找到 (尝试 0x%02X/0x%02X)", MPU9250_I2C_ADDR, MPU9250_I2C_ADDR_ALT);
        return ESP_ERR_NOT_FOUND;
    }

    /* 校验 WHOAMI */
    uint8_t who = 0;
    err = mpu_read_regs(mpu, MPU9250_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取 WHOAMI 失败: %s", esp_err_to_name(err));
        return err;
    }
    if (who != MPU9250_WHOAMI_MPU9250 && who != MPU9250_WHOAMI_MPU9255) {
        ESP_LOGE(TAG, "WHOAMI 不匹配: 0x%02X (期望 0x70/0x73)", who);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 软复位 -> 唤醒（PLL 陀螺 X 参考） */
    (void)mpu_write_reg(mpu, MPU9250_REG_PWR_MGMT_1, MPU9250_PWR_MGMT_1_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));  /* 数据手册要求软复位后 ≥50ms */
    err = mpu_write_reg(mpu, MPU9250_REG_PWR_MGMT_1, MPU9250_PWR_MGMT_1_WAKE_PLL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "唤醒 MPU9250 失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));  /* 唤醒后稳定时间 */

    /* 采样率与量程配置 */
    (void)mpu_write_reg(mpu, MPU9250_REG_SMPLRT_DIV, MPU9250_SMPLRT_DIV_100HZ);
    (void)mpu_write_reg(mpu, MPU9250_REG_CONFIG, 0x03);  /* DLPF 41Hz */
    err = mpu_write_reg(mpu, MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_4G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置加速度计量程失败: %s", esp_err_to_name(err));
        return err;
    }
    err = mpu_write_reg(mpu, MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_2000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置陀螺仪量程失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 关闭内部 I2C 主机并打开 Bypass，使 AK8963 直接挂在总线上 */
    (void)mpu_write_reg(mpu, MPU9250_REG_USER_CTRL, MPU9250_USER_CTRL_I2C_MST_OFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    err = mpu_write_reg(mpu, MPU9250_REG_INT_PIN_CFG, MPU9250_INT_PIN_CFG_BYPASS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置 I2C Bypass 失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  /* 旁路模式稳定 + AK8963 上电时间 */

    mpu->present = true;
    ESP_LOGI(TAG, "MPU9250 初始化成功 (0x%02X), WHOAMI=0x%02X", used_addr, who);

    /* 磁力计为可选，失败不影响 IMU 主体 */
    (void)mpu9250_init_mag(mpu, bus);
    return ESP_OK;
}

/* ---------- 读取 ---------- */

static esp_err_t mpu9250_read_mag(mpu9250_t *mpu, mpu9250_sample_t *s)
{
    /* 一次性读取 ST1(0x02) + 6 字节数据 + ST2(0x09) */
    uint8_t d[AK8963_BURST_BYTES] = {0};
    esp_err_t err = mag_read_regs(mpu, AK8963_REG_ST1, d, sizeof(d));
    if (err != ESP_OK) {
        return err;
    }

    /* ST2 bit3 = HOFL，磁力计溢出时数据不可信 */
    if ((d[7] & 0x08u) != 0u) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    int16_t mx = (int16_t)(((uint16_t)d[2] << 8) | d[1]);
    int16_t my = (int16_t)(((uint16_t)d[4] << 8) | d[3]);
    int16_t mz = (int16_t)(((uint16_t)d[6] << 8) | d[5]);

    s->mag_x = (float)mx * MPU9250_MAG_UT_PER_LSB;
    s->mag_y = (float)my * MPU9250_MAG_UT_PER_LSB;
    s->mag_z = (float)mz * MPU9250_MAG_UT_PER_LSB;
    return ESP_OK;
}

esp_err_t mpu9250_read(mpu9250_t *mpu, mpu9250_sample_t *sample)
{
    if (mpu == NULL || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mpu->present) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(sample, 0, sizeof(*sample));

    /* 加速度计：0x3B~0x40，±4g -> /8192 */
    uint8_t d[MPU9250_AXIS_BYTES] = {0};
    esp_err_t err = mpu_read_regs(mpu, MPU9250_REG_ACCEL_XOUT_H, d, sizeof(d));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度计失败: %s", esp_err_to_name(err));
        return err;
    }
    sample->acc_x = (float)(int16_t)(((uint16_t)d[0] << 8) | d[1]) / MPU9250_ACCEL_LSB_PER_G;
    sample->acc_y = (float)(int16_t)(((uint16_t)d[2] << 8) | d[3]) / MPU9250_ACCEL_LSB_PER_G;
    sample->acc_z = (float)(int16_t)(((uint16_t)d[4] << 8) | d[5]) / MPU9250_ACCEL_LSB_PER_G;

    /* 陀螺仪：0x43~0x48，±2000dps -> /16.384 */
    err = mpu_read_regs(mpu, MPU9250_REG_GYRO_XOUT_H, d, sizeof(d));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取陀螺仪失败: %s", esp_err_to_name(err));
        return err;
    }
    sample->gyro_x = (float)(int16_t)(((uint16_t)d[0] << 8) | d[1]) / MPU9250_GYRO_LSB_PER_DPS;
    sample->gyro_y = (float)(int16_t)(((uint16_t)d[2] << 8) | d[3]) / MPU9250_GYRO_LSB_PER_DPS;
    sample->gyro_z = (float)(int16_t)(((uint16_t)d[4] << 8) | d[5]) / MPU9250_GYRO_LSB_PER_DPS;

    /* 磁力计为可选 */
    if (mpu->mag_present) {
        if (mpu9250_read_mag(mpu, sample) != ESP_OK) {
            sample->mag_x = 0.0f;
            sample->mag_y = 0.0f;
            sample->mag_z = 0.0f;
        }
    }

    return ESP_OK;
}
