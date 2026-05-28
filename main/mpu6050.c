// mpu6050.c
#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MPU6050";

// I2C 引脚配置
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_FREQ_HZ 400000 // 400kHz
#define I2C_MASTER_TX_BUF_DIS 0
#define I2C_MASTER_RX_BUF_DIS 0
#define I2C_MASTER_TIMEOUT_MS 100

// MPU6050 寄存器地址
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

// 加速度数据输出寄存器（高8位+低8位）
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

// 陀螺仪数据输出寄存器
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

// 量程设置
#define ACCEL_RANGE_2G 0x00  // ±2g
#define ACCEL_RANGE_4G 0x08  // ±4g
#define ACCEL_RANGE_8G 0x10  // ±8g
#define ACCEL_RANGE_16G 0x18 // ±16g

#define GYRO_RANGE_250DPS 0x00  // ±250 °/s
#define GYRO_RANGE_500DPS 0x08  // ±500 °/s
#define GYRO_RANGE_1000DPS 0x10 // ±1000 °/s
#define GYRO_RANGE_2000DPS 0x18 // ±2000 °/s

// 当前使用的量程（可修改）
static uint8_t accel_range = ACCEL_RANGE_2G;
static uint8_t gyro_range = GYRO_RANGE_250DPS;

// 灵敏度缩放因子
static float accel_scale; // LSB/g
static float gyro_scale;  // LSB/(°/s)

// 陀螺仪零偏（校准用）
static float gyro_bias[3] = {0, 0, 0};

/**
 * @brief 写一个字节到 MPU6050 寄存器
 */
static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write error: %d", ret);
    }
    return ret;
}

/**
 * @brief 从 MPU6050 读取一个字节
 */
static esp_err_t mpu6050_read_byte(uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read error: %d", ret);
    }
    return ret;
}

/**
 * @brief 连续读取多个字节（用于加速度/陀螺仪数据）
 */
static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read bytes error: %d", ret);
    }
    return ret;
}

/**
 * @brief 根据量程计算缩放系数
 */
static void update_scales(void)
{
    // 加速度缩放 (LSB/g)
    switch (accel_range)
    {
    case ACCEL_RANGE_2G:
        accel_scale = 16384.0f;
        break;
    case ACCEL_RANGE_4G:
        accel_scale = 8192.0f;
        break;
    case ACCEL_RANGE_8G:
        accel_scale = 4096.0f;
        break;
    case ACCEL_RANGE_16G:
        accel_scale = 2048.0f;
        break;
    default:
        accel_scale = 16384.0f;
        break;
    }
    // 陀螺仪缩放 (LSB/°/s)
    switch (gyro_range)
    {
    case GYRO_RANGE_250DPS:
        gyro_scale = 131.0f;
        break;
    case GYRO_RANGE_500DPS:
        gyro_scale = 65.5f;
        break;
    case GYRO_RANGE_1000DPS:
        gyro_scale = 32.8f;
        break;
    case GYRO_RANGE_2000DPS:
        gyro_scale = 16.4f;
        break;
    default:
        gyro_scale = 131.0f;
        break;
    }
}

/**
 * @brief 初始化 I2C 总线
 */
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DIS,
                                       I2C_MASTER_TX_BUF_DIS, 0));
}

/**
 * @brief 检查 MPU6050 连接
 */
static bool mpu6050_check(void)
{
    uint8_t whoami = 0;
    esp_err_t ret = mpu6050_read_byte(MPU6050_WHO_AM_I, &whoami);
    if (ret != ESP_OK || whoami != 0x68)
    {
        ESP_LOGE(TAG, "MPU6050 not found! WHO_AM_I = 0x%02X", whoami);
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 detected (WHO_AM_I=0x%02X)", whoami);
    return true;
}

/**
 * @brief 配置 MPU6050 量程和唤醒
 */
static void mpu6050_config(void)
{
    // 唤醒传感器（清除睡眠位）
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 设置加速度量程
    mpu6050_write_byte(MPU6050_ACCEL_CONFIG, accel_range);
    // 设置陀螺仪量程
    mpu6050_write_byte(MPU6050_GYRO_CONFIG, gyro_range);

    update_scales();
    ESP_LOGI(TAG, "MPU6050 configured: Accel=%.0f LSB/g, Gyro=%.1f LSB/(°/s)",
             accel_scale, gyro_scale);
}

// 公共 API --------------------------------------------------------------

void mpu6050_init(void)
{
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!mpu6050_check())
    {
        ESP_LOGE(TAG, "MPU6050 init failed!");
        return;
    }
    mpu6050_config();
}

void mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    if (mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, buf, 6) != ESP_OK)
    {
        *ax = *ay = *az = 0;
        return;
    }
    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    *ax = raw_x / accel_scale;
    *ay = raw_y / accel_scale;
    *az = raw_z / accel_scale;
}

void mpu6050_read_gyro(float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    if (mpu6050_read_bytes(MPU6050_GYRO_XOUT_H, buf, 6) != ESP_OK)
    {
        *gx = *gy = *gz = 0;
        return;
    }
    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    *gx = raw_x / gyro_scale - gyro_bias[0];
    *gy = raw_y / gyro_scale - gyro_bias[1];
    *gz = raw_z / gyro_scale - gyro_bias[2];
}

void mpu6050_read_all(float *ax, float *ay, float *az,
                      float *gx, float *gy, float *gz)
{
    uint8_t buf[14]; // 加速度 6 字节 + 陀螺仪 6 字节 + 跳过温度 2 字节
    // 从加速度寄存器开始连续读取 14 字节（含陀螺仪）
    if (mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, buf, 14) != ESP_OK)
    {
        *ax = *ay = *az = *gx = *gy = *gz = 0;
        return;
    }
    // 加速度
    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    // 跳过温度 (buf[6], buf[7])
    // 陀螺仪
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    *ax = raw_ax / accel_scale;
    *ay = raw_ay / accel_scale;
    *az = raw_az / accel_scale;
    *gx = raw_gx / gyro_scale - gyro_bias[0];
    *gy = raw_gy / gyro_scale - gyro_bias[1];
    *gz = raw_gz / gyro_scale - gyro_bias[2];
}

void mpu6050_calibrate_gyro(void)
{
    ESP_LOGI(TAG, "Calibrating gyro... Keep MPU6050 still!");
    vTaskDelay(pdMS_TO_TICKS(100));

    float sum_x = 0, sum_y = 0, sum_z = 0;
    const int samples = 100;
    for (int i = 0; i < samples; i++)
    {
        uint8_t buf[6];
        if (mpu6050_read_bytes(MPU6050_GYRO_XOUT_H, buf, 6) == ESP_OK)
        {
            int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);
            sum_x += raw_x / gyro_scale;
            sum_y += raw_y / gyro_scale;
            sum_z += raw_z / gyro_scale;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    gyro_bias[0] = sum_x / samples;
    gyro_bias[1] = sum_y / samples;
    gyro_bias[2] = sum_z / samples;
    ESP_LOGI(TAG, "Gyro bias: X=%.2f, Y=%.2f, Z=%.2f (deg/s)",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}