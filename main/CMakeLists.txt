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
        imu_algo
        esp_driver_i2c
        esp_driver_gpio
        esp_timer
        esp_system
        freertos
)
# i2c_config.h 位于本组件（main/），引脚与 I2C_SCL_SPEED_HZ 在 app_main 中传入各驱动 init。
# imu_algo：从 stm32f103 移植的陀螺仪优化链（滤波 → 零偏 → 姿态），已在 imu_task 中接入。
