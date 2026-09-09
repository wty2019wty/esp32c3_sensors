/*
 * SSD1315 OLED 驱动实现（ESP-IDF v6.1 新版 I2C master API）
 */
#include "ssd1315.h"

#include <string.h>

#include "esp_log.h"
#include "font_6x8.h"

static const char *TAG = "ssd1315";

/* 初始化命令序列（任务书给定，SSD1315 兼容 SSD1306 指令集）。
 * 单条控制字节 0x00 后跟的全部字节均作为命令处理。 */
static const uint8_t s_init_cmds[] = {
    0xAE,           /* 关闭显示 */
    0x20, 0x00,     /* 水平寻址模式 */
    0xB0,           /* 页起始地址 0 */
    0xC8,           /* COM 扫描方向：反向 */
    0x00,           /* 低列地址 0 */
    0x10,           /* 高列地址 0 */
    0x40,           /* 显示起始行 0 */
    0x81, 0xCF,     /* 对比度 */
    0xA1,           /* 段重映射 */
    0xA6,           /* 正常显示 */
    0xA8, 0x3F,     /* 多路复用比 63 */
    0xA4,           /* 输出跟随 RAM */
    0xD3, 0x00,     /* 显示偏移 0 */
    0xD5, 0x80,     /* 时钟分频 */
    0xD9, 0xF1,     /* 预充电周期 */
    0xDA, 0x12,     /* COM 引脚配置 */
    0xDB, 0x40,     /* VCOMH 电压 */
    0x8D, 0x14,     /* 电荷泵使能 */
    0xAF,           /* 开启显示 */
};

/* 发送一批命令（首字节为控制字节 0x00） */
static esp_err_t ssd1315_send_cmds(ssd1315_t *oled, const uint8_t *cmds, size_t len)
{
    uint8_t tmp[1 + sizeof(s_init_cmds)];
    if (len + 1 > sizeof(tmp)) {
        return ESP_ERR_INVALID_SIZE;
    }
    tmp[0] = SSD1315_CTRL_CMD;
    memcpy(&tmp[1], cmds, len);
    return i2c_master_transmit(oled->dev, tmp, len + 1, SSD1315_I2C_TIMEOUT_MS);
}

/* 发送单条命令 */
static esp_err_t ssd1315_send_cmd(ssd1315_t *oled, uint8_t cmd)
{
    uint8_t tmp[2] = {SSD1315_CTRL_CMD, cmd};
    return i2c_master_transmit(oled->dev, tmp, sizeof(tmp), SSD1315_I2C_TIMEOUT_MS);
}

esp_err_t ssd1315_init(ssd1315_t *oled, i2c_master_bus_handle_t bus)
{
    if (oled == NULL || bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(oled, 0, sizeof(*oled));
    oled->present = false;

    esp_err_t err = i2c_master_probe(bus, SSD1315_I2C_ADDR, SSD1315_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1315 (0x%02X) 探测失败: %s", SSD1315_I2C_ADDR, esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1315_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &oled->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1315 添加设备失败: %s", esp_err_to_name(err));
        return err;
    }

    err = ssd1315_send_cmds(oled, s_init_cmds, sizeof(s_init_cmds));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1315 初始化命令失败: %s", esp_err_to_name(err));
        return err;
    }

    oled->present = true;
    ssd1315_clear(oled);
    err = ssd1315_flush(oled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1315 清屏失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SSD1315 初始化成功 (0x%02X, %dx%d)", SSD1315_I2C_ADDR, SSD1315_WIDTH, SSD1315_HEIGHT);
    return ESP_OK;
}

void ssd1315_clear(ssd1315_t *oled)
{
    if (oled == NULL) {
        return;
    }
    memset(oled->buf, 0, sizeof(oled->buf));
}

void ssd1315_draw_char(ssd1315_t *oled, uint8_t page, uint8_t col, char c)
{
    if (oled == NULL || page >= SSD1315_PAGES || col >= SSD1315_MAX_COLS) {
        return;
    }

    if (c < (char)FONT6X8_FIRST || c > (char)(FONT6X8_FIRST + FONT6X8_COUNT - 1)) {
        c = '?';
    }

    uint8_t idx = (uint8_t)(c - FONT6X8_FIRST);
    size_t off = (size_t)page * SSD1315_WIDTH + (size_t)col * SSD1315_CHAR_WIDTH;
    memcpy(&oled->buf[off], font_6x8[idx], SSD1315_CHAR_WIDTH);
}

void ssd1315_draw_string(ssd1315_t *oled, uint8_t page, uint8_t col, const char *str)
{
    if (oled == NULL || str == NULL) {
        return;
    }

    uint8_t c = col;
    while (*str != '\0' && c < SSD1315_MAX_COLS) {
        ssd1315_draw_char(oled, page, c, *str);
        c++;
        str++;
    }
}

esp_err_t ssd1315_flush(ssd1315_t *oled)
{
    if (oled == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!oled->present) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t page = 0; page < SSD1315_PAGES; page++) {
        /* 设置页地址与列地址（低 4 位 + 高 4 位） */
        esp_err_t err = ssd1315_send_cmd(oled, (uint8_t)(0xB0u | page));
        if (err != ESP_OK) {
            return err;
        }
        err = ssd1315_send_cmd(oled, 0x00);     /* 低列地址 = 0 */
        if (err != ESP_OK) {
            return err;
        }
        err = ssd1315_send_cmd(oled, 0x10);     /* 高列地址 = 0 */
        if (err != ESP_OK) {
            return err;
        }

        /* 发送 128 字节数据（首字节为数据控制字节 0x40） */
        uint8_t data[1 + SSD1315_WIDTH];
        data[0] = SSD1315_CTRL_DATA;
        memcpy(&data[1], &oled->buf[(size_t)page * SSD1315_WIDTH], SSD1315_WIDTH);
        err = i2c_master_transmit(oled->dev, data, sizeof(data), SSD1315_I2C_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}
