/*
 * SHT40 温湿度传感器驱动实现（ESP-IDF v6.1 新版 I2C master API）
 */
#include "sht40.h"

#include "esp_log.h"
#include "i2c_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht40";

uint8_t sht40_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = SHT40_CRC_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ SHT40_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

esp_err_t sht40_init(sht40_t *sht, i2c_master_bus_handle_t bus)
{
    if (sht == NULL || bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sht->dev = NULL;
    sht->present = false;

    /* 依次尝试主地址与备用地址 */
    static const uint8_t addrs[] = { SHT40_I2C_ADDR, SHT40_I2C_ADDR_ALT };
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        if (i2c_master_probe(bus, addrs[i], SHT40_I2C_TIMEOUT_MS) != ESP_OK) {
            continue;
        }

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = I2C_SCL_SPEED_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &sht->dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SHT40 添加设备失败: %s", esp_err_to_name(err));
            return err;
        }

        sht->present = true;
        ESP_LOGI(TAG, "SHT40 初始化成功 (0x%02X)", addrs[i]);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "SHT40 未找到 (尝试 0x%02X/0x%02X)", SHT40_I2C_ADDR, SHT40_I2C_ADDR_ALT);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t sht40_read(sht40_t *sht, float *temp_c, float *humi_rh)
{
    if (sht == NULL || temp_c == NULL || humi_rh == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sht->present) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 1. 发送高精度测量命令 */
    uint8_t cmd = SHT40_CMD_MEASURE_HIGH_PREC;
    esp_err_t err = i2c_master_transmit(sht->dev, &cmd, 1, SHT40_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "测量命令发送失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. 等待转换完成（>=10ms） */
    vTaskDelay(pdMS_TO_TICKS(SHT40_MEASURE_DELAY_MS));

    /* 3. 读取 6 字节原始数据 */
    uint8_t raw[SHT40_RAW_BYTES] = {0};
    err = i2c_master_receive(sht->dev, raw, sizeof(raw), SHT40_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取数据失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 4. CRC-8 校验（温度 2 字节 + 湿度 2 字节各带 1 字节 CRC） */
    if (sht40_crc8(&raw[0], 2) != raw[2]) {
        ESP_LOGE(TAG, "温度 CRC 校验失败");
        return ESP_ERR_INVALID_CRC;
    }
    if (sht40_crc8(&raw[3], 2) != raw[5]) {
        ESP_LOGE(TAG, "湿度 CRC 校验失败");
        return ESP_ERR_INVALID_CRC;
    }

    /* 5. 换算公式（Datasheet）：
     *    T  = -45 + 175 * raw_t / 65535
     *    RH = -6  + 125 * raw_h / 65535
     */
    uint16_t raw_t = (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    uint16_t raw_h = (uint16_t)(((uint16_t)raw[3] << 8) | raw[4]);

    *temp_c = -45.0f + 175.0f * ((float)raw_t / 65535.0f);

    float rh = -6.0f + 125.0f * ((float)raw_h / 65535.0f);
    if (rh > 100.0f) {
        rh = 100.0f;
    } else if (rh < 0.0f) {
        rh = 0.0f;
    }
    *humi_rh = rh;

    return ESP_OK;
}
