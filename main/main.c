#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "attitude_control.h"
#include "ble_control.h" // 替换原来的 car_manager.h

static const char *TAG = "MAIN";

void app_main(void)
{
    motor_init();
    mpu6050_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    attitude_init();    // 必须先初始化姿态（校准陀螺仪）
    ble_control_init(); // 启动 BLE 控制

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}