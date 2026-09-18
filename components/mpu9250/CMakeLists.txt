idf_component_register(
    SRCS "mpu9250.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        freertos
        log
)
