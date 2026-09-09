/*
 * SSD1315 OLED 驱动实现（ESP-IDF v6.1 新版 I2C master API）
 *
 * 点亮方式参考已验证工程 G:\esp32s3\ssd1315oled：
 *   - 初始化命令逐条发送（每条控制字节 0x00）
 *   - 刷新时使用水平寻址（0x21 列范围 / 0x22 页范围）
 *   - 一次性发送整帧 1024 字节数据
 */
#include "ssd1315.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "font_6x8.h"
#include "i2c_config.h"

static const char *TAG = "ssd1315";

/* 初始化命令序列（任务书给定，SSD1315 兼容 SSD1306 指令集） */
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

/* 发送单条命令：[0x00, cmd] */
static esp_err_t ssd1315_send_cmd(ssd1315_t *oled, uint8_t cmd)
{
    uint8_t buf[2] = {SSD1315_CTRL_CMD, cmd};
    return i2c_master_transmit(oled->dev, buf, sizeof(buf), SSD1315_I2C_TIMEOUT_MS);
}

/* 发送一段数据：[0x40, data...] */
static esp_err_t ssd1315_send_data(ssd1315_t *oled, const uint8_t *data, size_t len)
{
    uint8_t *buf = malloc(len + 1);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    buf[0] = SSD1315_CTRL_DATA;
    memcpy(buf + 1, data, len);

    esp_err_t ret = i2c_master_transmit(oled->dev, buf, len + 1, SSD1315_I2C_TIMEOUT_MS);
    free(buf);
    return ret;
}

esp_err_t ssd1315_init(ssd1315_t *oled, i2c_master_bus_handle_t bus)
{
    if (oled == NULL || bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(oled, 0, sizeof(*oled));
    oled->present = false;

    /* 初始化前复位总线，清理可能残留的忙状态 */
    (void)i2c_master_bus_reset(bus);

    /* 依次尝试主地址与备用地址 */
    static const uint8_t addrs[] = { SSD1315_I2C_ADDR, SSD1315_I2C_ADDR_ALT };
    uint8_t used_addr = 0;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        if (i2c_master_probe(bus, addrs[i], SSD1315_I2C_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = I2C_SCL_SPEED_HZ,
        };
        err = i2c_master_bus_add_device(bus, &dev_cfg, &oled->dev);
        if (err == ESP_OK) {
            used_addr = addrs[i];
            break;
        }
    }
    if (used_addr == 0) {
        ESP_LOGE(TAG, "SSD1315 未找到 (尝试 0x%02X/0x%02X)", SSD1315_I2C_ADDR, SSD1315_I2C_ADDR_ALT);
        return ESP_ERR_NOT_FOUND;
    }

    /* 逐条发送初始化命令 */
    for (size_t i = 0; i < sizeof(s_init_cmds); i++) {
        err = ssd1315_send_cmd(oled, s_init_cmds[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SSD1315 初始化命令 0x%02X 失败: %s", s_init_cmds[i], esp_err_to_name(err));
            return err;
        }
    }

    oled->present = true;
    ssd1315_clear(oled);
    err = ssd1315_flush(oled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1315 清屏失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SSD1315 初始化成功 (0x%02X, %dx%d)", used_addr, SSD1315_WIDTH, SSD1315_HEIGHT);
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

    esp_err_t err;

    /* 水平寻址：列 0~127 */
    err = ssd1315_send_cmd(oled, 0x21);
    if (err == ESP_OK) err = ssd1315_send_cmd(oled, 0x00);
    if (err == ESP_OK) err = ssd1315_send_cmd(oled, 0x7F);
    /* 页 0~7 */
    if (err == ESP_OK) err = ssd1315_send_cmd(oled, 0x22);
    if (err == ESP_OK) err = ssd1315_send_cmd(oled, 0x00);
    if (err == ESP_OK) err = ssd1315_send_cmd(oled, 0x07);
    if (err != ESP_OK) {
        return err;
    }

    /* 一次性发送整帧 1024 字节 */
    return ssd1315_send_data(oled, oled->buf, SSD1315_BUF_SIZE);
}
