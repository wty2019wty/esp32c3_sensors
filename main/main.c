/*
 * ESP32-C3 Super Mini 多传感器系统
 *   - SHT40   温湿度            (I2C 0x44)
 *   - BMP280  温度/气压/海拔    (I2C 0x76, GY-91 板载)
 *   - MPU9250 加速度/陀螺仪     (I2C 0x68, GY-91 板载)
 *   - AK8963  磁力计            (I2C 0x0C, MPU9250 内置)
 *   - SSD1315 128x64 OLED       (I2C 0x3C)
 *
 * 框架：ESP-IDF v6.1（新版 i2c_master API）
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bmp280.h"
#include "i2c_config.h"
#include "imu_bias_calib.h"
#include "imu_filter.h"
#include "mahony.h"
#include "mpu9250.h"
#include "sht40.h"
#include "ssd1315.h"

static const char *TAG = "main";

/* ---------------- 硬件与任务配置 ---------------- */
/* I2C 引脚与速率见 i2c_config.h */
#define I2C_TIMEOUT_MS          100             /* 所有 I2C 操作超时 */
#define I2C_GLITCH_IGNORE_CNT   7               /* 典型滤波值 */

#define SENSOR_TASK_PRIORITY    4
#define IMU_TASK_PRIORITY       5
#define DISPLAY_TASK_PRIORITY   3
#define TASK_STACK_SIZE         4096
#define SENSOR_PERIOD_MS        50              /* 温湿度/气压采样周期：50ms（20Hz） */
#define IMU_PERIOD_MS           10              /* IMU 采样周期：10ms（100Hz，对齐 MPU ODR） */
#define DISPLAY_PERIOD_MS       20              /* 刷新周期：20ms（约 50 FPS，脏页刷新） */

#define I2C_DEV_COUNT           4
#define I2C_ADDR_OLED           0x3C
#define I2C_ADDR_SHT40          0x44
#define I2C_ADDR_MPU9250        0x68
#define I2C_ADDR_BMP280         0x76

#define MAHONY_KP               1.0f
#define MAHONY_KI               0.0005f
#define MAHONY_KI_STILL         0.005f          /* 静止时提高 Ki，加速残余零偏收敛 */
/* ZUPT：静止且补偿后残差角速度低于该值时，融合输入置零，避免噪声积分成 Yaw 漂移 */
#define IMU_ZUPT_RESID_DPS      0.25f
#define IMU_ZUPT_RESID_SQ       (IMU_ZUPT_RESID_DPS * IMU_ZUPT_RESID_DPS)

/* IMU 前端滤波与零偏校准（移植自 stm32f103 提高陀螺仪精度的方案） */
#define IMU_SAMPLE_HZ           (1000.0f / IMU_PERIOD_MS)  /* 100Hz */
#define IMU_ACCEL_CUTOFF_HZ     30.0f           /* 加速度二阶低通截止频率 */
#define IMU_GYRO_CUTOFF_HZ      25.0f           /* 陀螺仪二阶低通截止频率 */
#define IMU_FILTER_WARMUP       200             /* 滤波器预热帧数（100Hz 下约 2s） */

#define DISPLAY_COLS            21
#define DISPLAY_ROWS            8

/* 设为 1 时只初始化 OLED 并显示测试图案，不初始化任何传感器，
 * 用于单独排查 OLED 点亮问题（排除其它模块对总线的干扰）。 */
#define OLED_SELF_TEST          0

/* ---------------- 全局传感器数据结构 ---------------- */
typedef struct {
    /* SHT40 */
    float sht40_temp;       /* ℃ */
    float sht40_humi;       /* %RH */
    bool  sht40_valid;

    /* BMP280 */
    float bmp280_temp;      /* ℃（必须计算，同时用于气压补偿） */
    float bmp280_press;     /* hPa */
    float bmp280_alt;       /* 海拔 m */
    int32_t t_fine;         /* BMP280 内部中间变量 */
    bool  bmp280_valid;

    /* MPU9250 */
    float acc_x, acc_y, acc_z;      /* g */
    float gyro_x, gyro_y, gyro_z;   /* °/s */
    float mag_x, mag_y, mag_z;      /* μT */
    float roll, pitch, yaw;         /* ° */
    bool  mpu9250_valid;
    bool  imu_calibrated;           /* 陀螺零偏校准是否完成 */

    /* 系统 */
    uint32_t uptime_sec;
    uint32_t heap_free_kb;
    bool i2c_devices[I2C_DEV_COUNT];    /* 0x3C, 0x44, 0x68, 0x76 在线状态 */
} sensor_data_t;

/* ---------------- 全局对象 ---------------- */
static i2c_master_bus_handle_t s_bus;
static sht40_t s_sht;
static bmp280_t s_bmp;
static mpu9250_t s_mpu;
static ssd1315_t s_oled;

static sensor_data_t s_data;
static SemaphoreHandle_t s_data_mutex;
static SemaphoreHandle_t s_i2c_mutex;       /* I2C 总线互斥锁，保护多任务并发访问 */

/* 扫描的 I2C 地址顺序与 i2c_devices[] 下标一一对应 */
static const uint8_t s_scan_addrs[I2C_DEV_COUNT] = {
    I2C_ADDR_OLED, I2C_ADDR_SHT40, I2C_ADDR_MPU9250, I2C_ADDR_BMP280,
};

/* ---------------- I2C 总线 ---------------- */

/**
 * @brief I2C 线电平自检：开内部上拉后读取 SDA/SCL 是否能为高
 *
 * 用于判断 GPIO8 板载 LED、短路或无上拉导致的 SDA 被拉低问题。
 * 必须在 i2c_new_master_bus() 之前调用。
 */
static void i2c_lines_selftest(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << I2C_SDA_GPIO) | (1ULL << I2C_SCL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C 线自检配置失败: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    int sda = gpio_get_level(I2C_SDA_GPIO);
    int scl = gpio_get_level(I2C_SCL_GPIO);
    ESP_LOGI(TAG, "I2C 线自检：SDA=%d SCL=%d (1=可拉高, 0=被强下拉/短路)", sda, scl);

    if (sda == 0 || scl == 0) {
        ESP_LOGW(TAG, "检测到 I2C 线无法拉高：检查板载 LED(GPIO8)/短路/上拉电阻");
    }
}

static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_CNT,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

/**
 * @brief 探测一个 I2C 地址，失败时复位总线并重试一次
 *
 * @param[in] addr 7 位地址
 * @return true 在线；false 未应答
 */
static bool i2c_probe_retry(uint8_t addr)
{
    if (i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
        return true;
    }
    /* 失败时复位总线（发送时钟脉冲释放被拉低的 SDA）后重试 */
    (void)i2c_master_bus_reset(s_bus);
    vTaskDelay(pdMS_TO_TICKS(5));
    return (i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK);
}

static void i2c_scan(void)
{
    /* 先复位一次总线，清理可能残留的忙状态 */
    (void)i2c_master_bus_reset(s_bus);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 全地址扫描，便于排查模块真实地址/接线问题 */
    ESP_LOGI(TAG, "全总线扫描 (0x08~0x77) ...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_probe_retry(addr)) {
            ESP_LOGI(TAG, "  发现 I2C 设备: 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "扫描完成，共发现 %d 个设备", found);

    /* 逐一检查期望的四个地址 */
    for (int i = 0; i < I2C_DEV_COUNT; i++) {
        bool online = i2c_probe_retry(s_scan_addrs[i]);
        s_data.i2c_devices[i] = online;
        if (online) {
            ESP_LOGI(TAG, "  期望设备 0x%02X: 在线", s_scan_addrs[i]);
        } else {
            ESP_LOGE(TAG, "  期望设备 0x%02X: 未找到（检查接线/供电/上拉/地址跳线）", s_scan_addrs[i]);
        }
    }
}

/* ---------------- 数值格式化辅助 ---------------- */

/**
 * @brief 将浮点数格式化为固定宽度、右对齐、2 位小数的字段
 *
 * 宽度固定可保证小数点位置不随数值变化而左右跳动；
 * 无效数据用 "---" 右对齐占位，同样保持位置稳定。
 *
 * @param[out] out   输出缓冲区
 * @param[in]  n     缓冲区大小
 * @param[in]  valid 数据是否有效
 * @param[in]  v     输入值
 * @param[in]  width 字段宽度（字符数）
 */
static void fmt_field(char *out, size_t n, bool valid, float v, int width)
{
    if (valid) {
        snprintf(out, n, "%*.*f", width, 2, (double)v);
    } else {
        snprintf(out, n, "%*s", width, "---");
    }
}

/* ---------------- 显示渲染 ---------------- */

/**
 * @brief 将一次数据快照渲染到 OLED（单页 8 行，不翻页）
 *
 * @param[in] d 传感器数据快照
 */
static void display_render(const sensor_data_t *d)
{
    char line[64];
    char a[16], b[16], c[16];

    /* 刷新时先清缓冲区，避免残影 */
    ssd1315_clear(&s_oled);

    /* I2C 在线设备数（Line 2 右侧复用） */
    int online = 0;
    for (int i = 0; i < I2C_DEV_COUNT; i++) {
        if (d->i2c_devices[i]) {
            online++;
        }
    }
    char i2c[12];
    if (online == I2C_DEV_COUNT) {
        snprintf(i2c, sizeof(i2c), "OK");
    } else {
        snprintf(i2c, sizeof(i2c), "%d/%d", online, I2C_DEV_COUNT);
    }

    /* Line 0: SHT40 温度/湿度（固定宽度，2 位小数） */
    fmt_field(a, sizeof(a), d->sht40_valid, d->sht40_temp, 6);
    fmt_field(b, sizeof(b), d->sht40_valid, d->sht40_humi, 6);
    snprintf(line, sizeof(line), "SHT %sC %s%%", a, b);
    ssd1315_draw_string(&s_oled, 0, 0, line);

    /* Line 1: BMP280 温度/气压（固定宽度，2 位小数） */
    fmt_field(a, sizeof(a), d->bmp280_valid, d->bmp280_temp, 6);
    fmt_field(b, sizeof(b), d->bmp280_valid, d->bmp280_press, 7);
    snprintf(line, sizeof(line), "BMP %s %shPa", a, b);
    ssd1315_draw_string(&s_oled, 1, 0, line);

    /* Line 2: BMP280 海拔 + I2C 状态（校准期间显示 CAL） */
    fmt_field(a, sizeof(a), d->bmp280_valid, d->bmp280_alt, 7);
    if (d->mpu9250_valid && !d->imu_calibrated) {
        snprintf(line, sizeof(line), "ALT %sm CAL", a);
    } else {
        snprintf(line, sizeof(line), "ALT %sm I2C:%s", a, i2c);
    }
    ssd1315_draw_string(&s_oled, 2, 0, line);

    /* Line 3: 加速度计（A，单位 g，固定宽度） */
    fmt_field(a, sizeof(a), d->mpu9250_valid, d->acc_x, 6);
    fmt_field(b, sizeof(b), d->mpu9250_valid, d->acc_y, 6);
    fmt_field(c, sizeof(c), d->mpu9250_valid, d->acc_z, 6);
    snprintf(line, sizeof(line), "A%s %s %s", a, b, c);
    ssd1315_draw_string(&s_oled, 3, 0, line);

    /* Line 4: 陀螺仪（G，单位 °/s，已零偏补偿，固定宽度） */
    fmt_field(a, sizeof(a), d->mpu9250_valid, d->gyro_x, 6);
    fmt_field(b, sizeof(b), d->mpu9250_valid, d->gyro_y, 6);
    fmt_field(c, sizeof(c), d->mpu9250_valid, d->gyro_z, 6);
    snprintf(line, sizeof(line), "G%s %s %s", a, b, c);
    ssd1315_draw_string(&s_oled, 4, 0, line);

    /* Line 5: 磁力计（M，单位 μT，固定宽度）；无磁力计时显示 --- */
    bool mag_valid = d->mpu9250_valid &&
                     !(d->mag_x == 0.0f && d->mag_y == 0.0f && d->mag_z == 0.0f);
    fmt_field(a, sizeof(a), mag_valid, d->mag_x, 6);
    fmt_field(b, sizeof(b), mag_valid, d->mag_y, 6);
    fmt_field(c, sizeof(c), mag_valid, d->mag_z, 6);
    snprintf(line, sizeof(line), "M%s %s %s", a, b, c);
    ssd1315_draw_string(&s_oled, 5, 0, line);

    /* Line 6: Mahony 姿态角（R/P/Y，单位 °，固定宽度） */
    bool att_valid = d->mpu9250_valid && d->imu_calibrated;
    fmt_field(a, sizeof(a), att_valid, d->roll, 6);
    fmt_field(b, sizeof(b), att_valid, d->pitch, 6);
    fmt_field(c, sizeof(c), att_valid, d->yaw, 6);
    snprintf(line, sizeof(line), "R%sP%sY%s", a, b, c);
    ssd1315_draw_string(&s_oled, 6, 0, line);

    /* Line 7: 运行时间 + 空闲堆 */
    uint32_t up = d->uptime_sec;
    snprintf(line, sizeof(line), "Up:%02u:%02u:%02u Heap:%luK",
             (unsigned)(up / 3600u), (unsigned)((up / 60u) % 60u), (unsigned)(up % 60u),
             (unsigned long)d->heap_free_kb);
    ssd1315_draw_string(&s_oled, 7, 0, line);

    esp_err_t err = ssd1315_flush(&s_oled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED 刷新失败: %s", esp_err_to_name(err));
        (void)i2c_master_bus_reset(s_bus);
    }
}

/* ---------------- 任务 ---------------- */

/**
 * @brief IMU 任务：100Hz 读取 MPU9250，经 二阶低通滤波 -> 零偏校准/跟踪 -> Mahony 融合
 *
 * 该流水线移植自 stm32f103 工程提高陀螺仪精度的方案：
 *   1) imu_filter        抑制高频噪声，稳定陀螺积分；
 *   2) imu_bias_calib    上电静止校准 + 运行期静止零偏跟踪，消除陀螺漂移；
 *   3) mahony_update     四元数姿态融合，输出 Roll/Pitch/Yaw。
 */
static void imu_task(void *arg)
{
    (void)arg;

    mahony_t ahrs;
    mahony_init(&ahrs, MAHONY_KP, MAHONY_KI);

    imu_filter_t filter;
    imu_filter_init(&filter, IMU_SAMPLE_HZ,
                    IMU_ACCEL_CUTOFF_HZ, IMU_GYRO_CUTOFF_HZ, IMU_FILTER_WARMUP);

    imu_bias_calib_t calib;
    imu_bias_calib_init(&calib);

    /* 校准完成前不喂融合器：避免未补偿零偏先把四元数积分脏 */
    bool fusion_armed = false;

    int64_t last_us = esp_timer_get_time();

    while (1) {
        mpu9250_sample_t raw;
        memset(&raw, 0, sizeof(raw));

        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
        bool mpu_ok = (mpu9250_read(&s_mpu, &raw) == ESP_OK);
        if (!mpu_ok) {
            (void)i2c_master_bus_reset(s_bus);
        }
        xSemaphoreGive(s_i2c_mutex);

        /* 计算实际采样间隔（补偿调度抖动） */
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.5f) {
            dt = (float)IMU_PERIOD_MS / 1000.0f;
        }

        mpu9250_sample_t filtered = raw;
        mpu9250_sample_t fused_in = raw;

        if (mpu_ok) {
            imu_filter_process(&raw, &filtered, &filter);
            imu_bias_calib_update(&calib, &filtered, &fused_in);

            bool calib_done = imu_bias_calib_is_done(&calib);
            if (calib_done && !fusion_armed) {
                /* 零偏就绪后再从单位四元数起步，丢弃校准期的积分历史 */
                mahony_init(&ahrs, MAHONY_KP, MAHONY_KI);
                fusion_armed = true;
            }

            if (fusion_armed) {
                bool still = imu_bias_calib_is_still(&calib);
                mahony_set_gains(&ahrs, MAHONY_KP,
                                 still ? MAHONY_KI_STILL : MAHONY_KI);

                float gx = fused_in.gyro_x;
                float gy = fused_in.gyro_y;
                float gz = fused_in.gyro_z;

                /* ZUPT：静止且残差很小时强制角速度为 0，抑制 Yaw 噪声积分 */
                if (still) {
                    float r2 = gx * gx + gy * gy + gz * gz;
                    if (r2 < IMU_ZUPT_RESID_SQ) {
                        gx = 0.0f;
                        gy = 0.0f;
                        gz = 0.0f;
                    }
                }

                mahony_update(&ahrs,
                              gx, gy, gz,
                              fused_in.acc_x, fused_in.acc_y, fused_in.acc_z,
                              fused_in.mag_x, fused_in.mag_y, fused_in.mag_z,
                              dt);
            }
        }

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        mahony_get_euler(&ahrs, &roll, &pitch, &yaw);

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        if (mpu_ok) {
            s_data.acc_x = fused_in.acc_x;
            s_data.acc_y = fused_in.acc_y;
            s_data.acc_z = fused_in.acc_z;
            s_data.gyro_x = fused_in.gyro_x;
            s_data.gyro_y = fused_in.gyro_y;
            s_data.gyro_z = fused_in.gyro_z;
            s_data.mag_x = fused_in.mag_x;
            s_data.mag_y = fused_in.mag_y;
            s_data.mag_z = fused_in.mag_z;
            s_data.roll = roll;
            s_data.pitch = pitch;
            s_data.yaw = yaw;
        }
        s_data.mpu9250_valid = mpu_ok;
        s_data.imu_calibrated = imu_bias_calib_is_done(&calib);
        xSemaphoreGive(s_data_mutex);

        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/**
 * @brief 环境传感器任务：每 50ms 读取 SHT40 与 BMP280，并更新系统信息
 */
static void sensor_task(void *arg)
{
    (void)arg;

    while (1) {
        /* SHT40 */
        float sht_t = 0.0f, sht_h = 0.0f;
        /* BMP280（温度 -> t_fine -> 气压） */
        float bmp_t = 0.0f, bmp_p = 0.0f, bmp_a = 0.0f;
        int32_t t_fine = 0;
        bool sht_ok, bmp_ok;

        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
        sht_ok = (sht40_read(&s_sht, &sht_t, &sht_h) == ESP_OK);
        bmp_ok = (bmp280_read(&s_bmp, &bmp_t, &bmp_p, &bmp_a, &t_fine) == ESP_OK);

        /* 任一读取失败时复位总线，释放可能被拉低的 SDA */
        if (!sht_ok || !bmp_ok) {
            (void)i2c_master_bus_reset(s_bus);
        }
        xSemaphoreGive(s_i2c_mutex);

        int64_t now_us = esp_timer_get_time();

        /* 加锁写入全局数据 */
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);

        s_data.sht40_temp = sht_t;
        s_data.sht40_humi = sht_h;
        s_data.sht40_valid = sht_ok;

        s_data.bmp280_temp = bmp_t;
        s_data.bmp280_press = bmp_p;
        s_data.bmp280_alt = bmp_a;
        s_data.t_fine = t_fine;
        s_data.bmp280_valid = bmp_ok;

        s_data.uptime_sec = (uint32_t)(now_us / 1000000);
        s_data.heap_free_kb = esp_get_free_heap_size() / 1024u;

        xSemaphoreGive(s_data_mutex);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/**
 * @brief 显示任务：按 DISPLAY_PERIOD_MS 周期刷新 OLED（默认 100ms）
 */
static void display_task(void *arg)
{
    (void)arg;

    while (1) {
        sensor_data_t snapshot;
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        snapshot = s_data;
        xSemaphoreGive(s_data_mutex);

        if (s_oled.present) {
            xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
            display_render(&snapshot);
            xSemaphoreGive(s_i2c_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

/* ---------------- OLED 单独自检 ---------------- */
#if OLED_SELF_TEST
/**
 * @brief 仅测试 OLED：画棋盘格 + 文字，不初始化任何传感器
 */
static void oled_self_test(void)
{
    if (ssd1315_init(&s_oled, s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "OLED 自检初始化失败");
        return;
    }

    ssd1315_clear(&s_oled);
    for (uint8_t p = 0; p < SSD1315_PAGES; p++) {
        for (uint8_t c = 0; c < SSD1315_WIDTH; c++) {
            if (((p + c) & 1u) != 0u) {
                s_oled.buf[(size_t)p * SSD1315_WIDTH + c] = 0xFF;
            }
        }
    }
    ssd1315_draw_string(&s_oled, 0, 0, "OLED TEST OK");
    if (ssd1315_flush(&s_oled) == ESP_OK) {
        ESP_LOGI(TAG, "OLED 自检成功");
    } else {
        ESP_LOGE(TAG, "OLED 自检刷新失败");
    }
}
#endif

/* ---------------- 入口 ---------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 多传感器系统启动");

    i2c_lines_selftest();

    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C 总线初始化完成 (SDA=GPIO%d, SCL=GPIO%d, %dHz)",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_SCL_SPEED_HZ);

    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "互斥锁创建失败");
        return;
    }
    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "I2C 互斥锁创建失败");
        return;
    }
    memset(&s_data, 0, sizeof(s_data));

#if OLED_SELF_TEST
    oled_self_test();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    /* 等待各模块上电稳定后再扫描 */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 1. 扫描总线，记录设备在线状态 */
    i2c_scan();

    /* 2. 初始化各传感器（失败不阻塞，对应显示行显示 ---） */
    if (sht40_init(&s_sht, s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "SHT40 初始化失败，相关显示将标记为 ---");
    }
    if (bmp280_init(&s_bmp, s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 初始化失败，相关显示将标记为 ---");
    }
    if (mpu9250_init(&s_mpu, s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 初始化失败，相关显示将标记为 ---");
    }
    if (ssd1315_init(&s_oled, s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "SSD1315 初始化失败，将无法显示");
    }

    /* 3. 创建任务 */
    BaseType_t ok;
    ok = xTaskCreate(imu_task, "imu_task", TASK_STACK_SIZE, NULL,
                     IMU_TASK_PRIORITY, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "imu_task 创建失败");
        return;
    }
    ok = xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SIZE, NULL,
                     SENSOR_TASK_PRIORITY, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "sensor_task 创建失败");
        return;
    }
    ok = xTaskCreate(display_task, "display_task", TASK_STACK_SIZE, NULL,
                     DISPLAY_TASK_PRIORITY, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "display_task 创建失败");
        return;
    }

    ESP_LOGI(TAG, "任务已启动：imu_task(prio %d, %dHz) / sensor_task(prio %d) / display_task(prio %d)",
             IMU_TASK_PRIORITY, (int)IMU_SAMPLE_HZ, SENSOR_TASK_PRIORITY, DISPLAY_TASK_PRIORITY);
}
