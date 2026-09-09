# ESP32-C3 Super Mini 多传感器项目

基于 **ESP-IDF v6.1**（新版 `i2c_master` API）的完整工程，在 128×64 SSD1315 OLED
单页显示 SHT40、GY-91（BMP280 + MPU9250 + AK8963）的全部数据，不翻页。

## 1. 硬件接线

| 信号 | GPIO | 说明 |
| --- | --- | --- |
| SDA | GPIO8 | I2C 数据线 |
| SCL | GPIO9 | I2C 时钟线 |
| VCC | 3.3V | 所有模块供电（严禁 5V） |
| GND | GND | 共地 |

I2C 地址：

| 器件 | 地址 | 功能 |
| --- | --- | --- |
| SHT40 | 0x44 | 温湿度 |
| BMP280 | 0x76 | 温度 + 气压 + 海拔（GY-91） |
| MPU9250 | 0x68 | 加速度 + 陀螺仪（GY-91） |
| AK8963 | 0x0C | 磁力计（MPU9250 内置，需 I2C Bypass） |
| SSD1315 | 0x3C | 128×64 OLED（兼容 SSD1306 指令集） |

## 2. 目录结构

```
esp32c3_sensors/
├── CMakeLists.txt
├── sdkconfig.defaults
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml
    ├── main.c              # I2C 初始化、设备扫描、FreeRTOS 任务、显示布局
    ├── sht40.c / sht40.h   # SHT40 温湿度（CRC-8 校验）
    ├── bmp280.c / bmp280.h # BMP280 温度/气压/海拔（t_fine 耦合）
    ├── mpu9250.c / mpu9250.h # MPU9250 加速度/陀螺仪 + AK8963 磁力计
    ├── ssd1315.c / ssd1315.h # OLED 驱动（1024 字节帧缓冲 + 6×8 字体）
    ├── mahony.c / mahony.h # Mahony 姿态融合
    └── font_6x8.h          # 6×8 ASCII 点阵字体（96 字形）
```

## 3. 构建与烧录

```powershell
D:\esp\v6.1\esp-idf\export.ps1
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

> 若控制台出现 Unicode 警告，可先执行 `$env:PYTHONUTF8=1`。
> 编译默认启用 `-Wall -Werror`，本工程已保证零 warning。

## 4. 显示布局（单页，21 字符 × 8 行）

```
SHT:25.3C 56.2%
BMP:24.8C 1013hPa 32m
ACC .12 -.05 .99 g
GYR  12   46   -5 d/s
MAG 234 -156  412 uT
R:12.3 P:-3.1 Y:45.6
I2C:OK 400kHz
Up:00:05:23 Heap:123K
```

- 刷新时先清空帧缓冲再重写，避免残影。
- 传感器离线时对应行显示 `---`，程序不会卡死。
- 每 100ms 采集一次，每 1s 刷新一次显示。

## 5. 已知限制说明

1. **I2C 总线长度**
   - 400kHz 下总线走线建议 ≤ 30cm，且尽量短、远离高频/电源走线。
   - 线缆过长或电容过大时会出现 NACK / 数据错乱，可降低到 100kHz。

2. **上拉电阻**
   - 代码仅开启了 ESP32-C3 内部弱上拉（`enable_internal_pullup = true`），
     在 400kHz 下通常不足以保证波形质量。
   - **强烈建议** SDA/SCL 各外接 4.7kΩ 上拉到 3.3V（GY-91、OLED 模块大多已自带，
     但仍需确认）。缺失上拉会表现为设备随机掉线。

3. **供电**
   - 所有模块必须使用 3.3V，严禁 5V，否则可能损坏 ESP32-C3 的 IO。
   - GY-91 同时含气压计与九轴 IMU，峰值电流较大，建议电源加 10µF~100µF 去耦。

4. **磁力计校准**
   - 本工程未做硬铁/软铁校准，AK8963 的 μT 绝对值和 Yaw 角会受周围铁磁材料、
     电机、走线电流影响而产生偏移。
   - 工程内部只使用了出厂 ASAX/AY/AZ 之外未做灵敏度修正，如需高精度航向，
     请先做 8 字校准并写入硬铁偏移/软铁矩阵。

5. **AK8963 工作模式与灵敏度**
   - AK8963 `CNTL1` 的 bit4 为输出位宽选择（1 = 16 位）。任务书给出的 `0x06`
     实为 14 位模式（灵敏度 0.6 µT/LSB），与 `0.15 µT/LSB` 不匹配。
   - 本工程采用 **`0x16`（16 位连续测量模式 2）+ 0.15 µT/LSB**，读数正确。
     若需严格按 `0x06`，请同时把 `MPU9250_MAG_UT_PER_LSB` 改为 `0.6f`。

6. **BMP280 海拔**
   - 海拔公式以海平面标准大气压 101325 Pa 为基准，实际天气变化会导致
     数十米误差；如需精确海拔请根据当地气压修正基准值。

7. **温度/气压耦合**
   - 气压补偿依赖温度补偿得到的 `t_fine`，因此每次读取都先算温度。
     若温度校准异常导致 `t_fine` 不可用，气压同样会被标记为无效。

8. **SSD1315 兼容性**
   - SSD1315 指令集兼容 SSD1306，本驱动使用任务书给定的初始化序列。
     个别批次的 SSD1315 需要将对比度（0x81）或预充电（0xD9）微调。

9. **姿态融合**
   - Mahony 的 Kp=1.0、Ki=0.0 为通用值，磁力计未校准时建议仅参考 Roll/Pitch，
     Yaw 仅作趋势观察。
   - 采集周期为 100ms（10Hz），快速旋转时陀螺仪积分误差会增大。

10. **共享数据结构**
    - `sensor_task` 与 `display_task` 通过 `SemaphoreHandle_t` 互斥锁保护
      `sensor_data_t`；I2C 总线访问由 ESP-IDF 新版 I2C master 驱动内部串行化。

## 6. 实测结论（ESP32-C3 Super Mini + GY-91 + SHT40 + SSD1315）

本工程在实机上调试得到以下结论，供接线参考：

1. **OLED 供电务必确认**
   - 单独测试时曾因忘记给 OLED 接 VCC 导致 0x3C 扫不到；务必确认 OLED 的 VCC 接 3.3V。
   - 若高码率（400kHz）下出现长数据写入超时，再优先检查上拉电阻与线材长度。
   - 降速（`I2C_SCL_SPEED_HZ` 改 100000/10000）可作为快速排查手段。

2. **GY-91 实际为 MPU6500 + BMP280**
   - 读取 WHOAMI 返回 **0x70（MPU6500）**，而非 0x71（MPU9250），且 0x0C 无 AK8963。
   - 这是 GY-91 常见的“标 MPU9250 实为 MPU6500”版本；加速度/陀螺仪/姿态可用，
     **无磁力计**，磁力计行显示 `MAG --- --- --- uT`。
   - 驱动已兼容 0x70/0x71/0x73 三种 WHOAMI。

3. **I2C 时钟是“按设备”设置的**
   - ESP-IDF v6.1 新版驱动的时钟在 `i2c_device_config_t.scl_speed_hz`，总线配置结构体
     没有时钟字段；因此统一改速必须改 `main/i2c_config.h` 的 `I2C_SCL_SPEED_HZ`。

