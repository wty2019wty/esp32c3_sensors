# IMU 算法（滤波 / 零偏校准 / Mahony 姿态融合）
# 当前未接入主固件：main 未 REQUIRES 本组件，未被引用的符号不会链入 ELF。
# 重构时在 main/CMakeLists.txt 的 REQUIRES 中追加 `imu_algo`，并 include 对应头文件即可。
idf_component_register(
    SRCS
        "imu_filter.c"
        "imu_bias_calib.c"
        "mahony.c"
    INCLUDE_DIRS "include"
    REQUIRES
        mpu9250
        esp_common
        log
)
