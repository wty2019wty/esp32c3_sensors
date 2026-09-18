idf_component_register(
    SRCS "ssd1315.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES
        esp_driver_i2c
        esp_common
        log
)
