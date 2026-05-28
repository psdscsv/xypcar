#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "attitude_control.h"
#include "ble_control.h"
#include "car_control.h" // 新增

static const char *TAG = "MAIN";

void app_main(void)
{
    motor_init();
    mpu6050_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    attitude_init();    // 姿态解算初始化
    car_control_init(); // 启动控制任务（闭环+电机输出）
    ble_control_init(); // 启动 BLE 接收（只解析指令）

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}