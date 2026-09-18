idf_component_register(
    SRCS
        "imu_filter.c"
        "imu_bias_calib.c"
        "imu_attitude.c"
        "imu_algo.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        mpu9250
)
# 从 stm32f103 移植的 IMU 优化链：二阶低通 + 零偏校准/跟踪 + Mahony 六轴姿态。
