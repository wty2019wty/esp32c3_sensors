idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        sht40
        bmp280
        mpu9250
        ssd1315
        esp_driver_i2c
        esp_driver_gpio
        esp_timer
        esp_system
        freertos
)
# i2c_config.h 位于本组件（main/），引脚与 I2C_SCL_SPEED_HZ 在 app_main 中传入各驱动 init。
# imu_algo 刻意未列入 REQUIRES：组件源码在 components/imu_algo，但未被引用，
# 符号不会链入固件。接入姿态融合时在此追加 `imu_algo` 即可。
