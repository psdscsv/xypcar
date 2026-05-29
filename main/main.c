#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "attitude_control.h"
#include "car_control.h"
#include "ble_control.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    motor_init();
    mpu6050_init();
    encoder_init(); // 编码器必须先初始化
    vTaskDelay(pdMS_TO_TICKS(100));

    attitude_init();    // 姿态解算（校准陀螺仪）
    car_control_init(); // 启动控制任务（速度+姿态）
    ble_control_init(); // 启动 BLE 接收（可选）

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}