// mpu6050.h
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// MPU6050 默认 I2C 地址 (AD0 接地或悬空)
#define MPU6050_ADDR 0x68

    // 初始化 MPU6050（配置 I2C 总线、唤醒传感器、设置量程）
    void mpu6050_init(void);

    // 读取三轴加速度数据（单位：g）
    // 结果存储在 accel_x, accel_y, accel_z 中，范围约 ±2g / ±4g / ±8g / ±16g（取决于配置）
    void mpu6050_read_accel(float *ax, float *ay, float *az);

    // 读取三轴角速度数据（单位：度/秒）
    // 结果存储在 gyro_x, gyro_y, gyro_z 中，范围约 ±250 / ±500 / ±1000 / ±2000 dps（取决于配置）
    void mpu6050_read_gyro(float *gx, float *gy, float *gz);

    // 同时读取加速度和角速度
    void mpu6050_read_all(float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz);

    // 校准陀螺仪（读取 100 次求平均，减去零偏）
    void mpu6050_calibrate_gyro(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H