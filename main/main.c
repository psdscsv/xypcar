#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "attitude_control.h"
#include "car_control.h"
#include "wifi_manager.h"      // 新增，用于 WiFi 配网
#include "web_control.h"       // 新增
#include "ble_control.h"     // 若不需要 BLE 可注释

static const char *TAG = "MAIN";

void app_main(void) {
    // 1. 硬件初始化
    motor_init();
    mpu6050_init();
    encoder_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. 姿态解算（含陀螺仪校准） 
    attitude_init();

    // 3. 小车闭环控制任务（速度+姿态）
    car_control_init();

    // 4. WiFi 管理（启动 AP 配网或连接保存的 WiFi）
    wifi_init();                // 内部会启动 HTTP 服务器（若进入 AP 模式）

    // 5. 注册 Web 遥控页面和 API
    web_control_init();

    // 可选：同时启用 BLE 控制
    ble_control_init();

    ESP_LOGI(TAG, "System ready. Web control: http://192.168.4.1/control");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}