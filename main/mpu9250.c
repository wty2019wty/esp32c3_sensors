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

    /* WIA 校验通过后：软复位 -> 掉电 -> 进 Fuse ROM 读出厂灵敏度调整值 */
    err = mag_write_reg(mpu, AK8963_REG_CNTL2, AK8963_CNTL2_SRST);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 软复位失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    err = mag_write_reg(mpu, AK8963_REG_CNTL1, AK8963_CNTL1_POWER_DOWN);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 掉电失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 进 Fuse ROM 模式，读出厂灵敏度调整值 */
    err = mag_write_reg(mpu, AK8963_REG_CNTL1, AK8963_CNTL1_FUSE_ROM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 进入 Fuse ROM 失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t asa[3] = {0};
    err = mag_read_regs(mpu, AK8963_REG_ASAX, asa, sizeof(asa));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "读取 AK8963 Fuse ROM 失败: %s", esp_err_to_name(err));
        return err;
    }
    mpu->mag_adj[0] = (float)(asa[0] - 128) / 256.0f + 1.0f;
    mpu->mag_adj[1] = (float)(asa[1] - 128) / 256.0f + 1.0f;
    mpu->mag_adj[2] = (float)(asa[2] - 128) / 256.0f + 1.0f;

    /* 返回掉电，再进入连续测量模式 2（100Hz，16 位输出） */
    err = mag_write_reg(mpu, AK8963_REG_CNTL1, AK8963_CNTL1_POWER_DOWN);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 掉电失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    err = mag_write_reg(mpu, AK8963_REG_CNTL1, AK8963_CNTL1_CONT_MODE2_16BIT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 配置连续测量失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    mpu->mag_present = true;
    ESP_LOGI(TAG, "AK8963 初始化成功 (0x%02X), WIA=0x%02X, ASA=[%d %d %d]",
             AK8963_I2C_ADDR, wia, asa[0], asa[1], asa[2]);
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
    if (who != MPU9250_WHOAMI_MPU9250 && who != MPU9250_WHOAMI_MPU9255 &&
        who != MPU9250_WHOAMI_MPU6500) {
        ESP_LOGE(TAG, "WHOAMI 不匹配: 0x%02X (期望 0x70/0x71/0x73)", who);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 仅 0x71/0x73 内置 AK8963 磁力计；0x70 为 MPU6500，退化为六轴 */
    const bool has_mag = (who == MPU9250_WHOAMI_MPU9250 || who == MPU9250_WHOAMI_MPU9255);
    if (!has_mag) {
        ESP_LOGW(TAG, "检测到 MPU6500 (WHOAMI=0x70)：无 AK8963 磁力计，退化为六轴 IMU");
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

    /* 采样率与量程配置：硬件 DLPF 先压噪，再由软件二阶低通细滤 */
    (void)mpu_write_reg(mpu, MPU9250_REG_SMPLRT_DIV, MPU9250_SMPLRT_DIV_100HZ);
    (void)mpu_write_reg(mpu, MPU9250_REG_CONFIG, MPU9250_CONFIG_DLPF_G20HZ);
    err = mpu_write_reg(mpu, MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_4G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置加速度计量程失败: %s", esp_err_to_name(err));
        return err;
    }
    err = mpu_write_reg(mpu, MPU9250_REG_ACCEL_CONFIG2, MPU9250_ACCEL_CONFIG2_DLPF_A21);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置加速度 DLPF 失败: %s", esp_err_to_name(err));
        return err;
    }
    err = mpu_write_reg(mpu, MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_500);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置陀螺仪量程失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 关闭内部 I2C 主机并打开 Bypass，使 AK8963 直接挂在总线上 */
    err = mpu_write_reg(mpu, MPU9250_REG_USER_CTRL, MPU9250_USER_CTRL_I2C_MST_OFF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "关闭 I2C Master 失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    err = mpu_write_reg(mpu, MPU9250_REG_INT_PIN_CFG, MPU9250_INT_PIN_CFG_BYPASS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置 I2C Bypass 失败: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  /* 旁路模式稳定 + AK8963 上电时间 */
    
    /* 调试：读回寄存器验证写入 */
    uint8_t user_ctrl = 0, int_pin_cfg = 0;
    (void)mpu_read_regs(mpu, MPU9250_REG_USER_CTRL, &user_ctrl, 1);
    (void)mpu_read_regs(mpu, MPU9250_REG_INT_PIN_CFG, &int_pin_cfg, 1);
    ESP_LOGI(TAG, "寄存器验证: USER_CTRL=0x%02X, INT_PIN_CFG=0x%02X", user_ctrl, int_pin_cfg);
    if ((int_pin_cfg & 0x02) == 0) {
        ESP_LOGW(TAG, "警告: BYPASS_EN 位未设置成功!");
    }

    mpu->present = true;
    ESP_LOGI(TAG, "MPU9250 初始化成功 (0x%02X), WHOAMI=0x%02X", used_addr, who);

    /* 磁力计为可选，失败不影响 IMU 主体；MPU6500 无磁力计直接跳过 */
    if (has_mag) {
        (void)mpu9250_init_mag(mpu, bus);
    } else {
        /* 诊断：旁路后探测常见磁力计地址（AK8963/QMC5883L/HMC5883L），
         * 用于判断山寨板是否外挂了独立磁力计。 */
        static const uint8_t mag_probe[] = {0x0C, 0x0D, 0x1E};
        for (size_t i = 0; i < sizeof(mag_probe); i++) {
            if (i2c_master_probe(bus, mag_probe[i], MPU9250_I2C_TIMEOUT_MS) == ESP_OK) {
                ESP_LOGW(TAG, "旁路上发现疑似磁力计: 0x%02X（需另行适配）", mag_probe[i]);
            }
        }
    }
    return ESP_OK;
}

/* ---------- 读取 ---------- */

static esp_err_t mpu9250_read_mag(mpu9250_t *mpu, mpu9250_sample_t *s)
{
    /* 等待数据就绪（DRDY=ST1 bit0），最多约 15ms。
     * 100Hz 连续模式下新数据每 10ms 一个，读到旧/混合数据的概率极低。 */
    esp_err_t err;
    uint8_t st1 = 0;
    for (int i = 0; i < 15; i++) {
        err = mag_read_regs(mpu, AK8963_REG_ST1, &st1, 1);
        if (err != ESP_OK) {
            return err;
        }
        if ((st1 & AK8963_ST1_DRDY) != 0u) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if ((st1 & AK8963_ST1_DRDY) == 0u) {
        return ESP_ERR_TIMEOUT;  /* 15ms 内无新数据 */
    }

    /* 数据小端序：HXL(0x03)..HZH(0x08) + ST2(0x09)，一次突发读 7 字节 */
    uint8_t d[AK8963_BURST_BYTES] = {0};
    err = mag_read_regs(mpu, AK8963_REG_HXL, d, sizeof(d));
    if (err != ESP_OK) {
        return err;
    }

    /* ST2 bit3 = HOFL，磁力计溢出时数据不可信 */
    if ((d[6] & AK8963_ST2_HOFL) != 0u) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    int16_t mx = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
    int16_t my = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
    int16_t mz = (int16_t)(((uint16_t)d[5] << 8) | d[4]);

    s->mag_x = (float)mx * MPU9250_MAG_UT_PER_LSB * mpu->mag_adj[0];
    s->mag_y = (float)my * MPU9250_MAG_UT_PER_LSB * mpu->mag_adj[1];
    s->mag_z = (float)mz * MPU9250_MAG_UT_PER_LSB * mpu->mag_adj[2];
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

    /*
     * 加速度 + 温度 + 陀螺仪一次突发读（0x3B~0x48，14 字节）。
     * 相比分两次读：
     *   1) 三者来自同一采样时刻，姿态融合不再受读间延迟影响；
     *   2) 减少一次 I2C 事务，100Hz 周期余量更大。
     */
    uint8_t d[MPU9250_BURST_BYTES] = {0};
    esp_err_t err = mpu_read_regs(mpu, MPU9250_REG_ACCEL_XOUT_H, d, sizeof(d));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取 IMU 突发数据失败: %s", esp_err_to_name(err));
        return err;
    }

    sample->acc_x = (float)(int16_t)(((uint16_t)d[0] << 8) | d[1]) / MPU9250_ACCEL_LSB_PER_G;
    sample->acc_y = (float)(int16_t)(((uint16_t)d[2] << 8) | d[3]) / MPU9250_ACCEL_LSB_PER_G;
    sample->acc_z = (float)(int16_t)(((uint16_t)d[4] << 8) | d[5]) / MPU9250_ACCEL_LSB_PER_G;

    int16_t temp_raw = (int16_t)(((uint16_t)d[6] << 8) | d[7]);
    sample->temp_c = (float)temp_raw / MPU9250_TEMP_SENSITIVITY + MPU9250_TEMP_OFFSET_DEGC;

    sample->gyro_x = (float)(int16_t)(((uint16_t)d[8] << 8) | d[9]) / MPU9250_GYRO_LSB_PER_DPS;
    sample->gyro_y = (float)(int16_t)(((uint16_t)d[10] << 8) | d[11]) / MPU9250_GYRO_LSB_PER_DPS;
    sample->gyro_z = (float)(int16_t)(((uint16_t)d[12] << 8) | d[13]) / MPU9250_GYRO_LSB_PER_DPS;

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
