// car_control.c
#include "car_control.h"
#include "motor_control.h"
#include "encoder.h"
#include "attitude_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "CarCtrl";

static car_control_params_t s_params = {
    .stop = 1,
    .target_speed = 0,
    .target_turn = 0,
    .turn_gain = 0.5f,
    .speed_pid_kp = 1.2f,
    .speed_pid_ki = 0.2f,
    .speed_pid_kd = 0.05f,
};

static SemaphoreHandle_t s_params_mutex = NULL;

#define CONTROL_PERIOD_MS 20   // 50Hz 控制周期

static void control_task(void *pvParameters) {
    car_control_params_t params;
    float left_speed, right_speed;
    float target_linear_speed_ms;      // 单位 m/s
    float target_angular_rate_dps;     // 单位 °/s
    float left_out, right_out;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        car_control_get_params(&params);

        if (params.stop) {
            motor_set_speed(0, 0);
            attitude_clean_pid();   // 重置所有积分项
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        // 获取当前轮速（来自编码器）
        encoder_get_speed(&left_speed, &right_speed);

        // 将目标值转换为物理单位
        target_linear_speed_ms = params.target_speed * 0.01f;   // 因为 target_speed 单位是 0.01m/s
        target_angular_rate_dps = params.target_turn;           // 假设 target_turn 直接是 °/s，若非则需映射

        // 调用级联控制核心
        attitude_stabilize_with_speed(target_linear_speed_ms, target_angular_rate_dps,
                                      left_speed, right_speed,
                                      &left_out, &right_out);

        // 设置电机
        motor_set_speed(left_out, right_out);

        // 调试日志（可降级为 ESP_LOGV）
        ESP_LOGD(TAG, "v_tgt=%.2f m/s, turn_tgt=%.1f °/s, left=%.1f%%, right=%.1f%%",
                 target_linear_speed_ms, target_angular_rate_dps, left_out, right_out);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void car_control_init(void) {
    s_params_mutex = xSemaphoreCreateMutex();
    if (s_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // 将 BLE 传入的 PID 参数同步到姿态控制模块
    attitude_set_speed_pid(s_params.speed_pid_kp, s_params.speed_pid_ki, s_params.speed_pid_kd);
    // 偏航环默认 P 参数（可调）
    attitude_set_yaw_rate_pid(0.5f, 0.0f, 0.0f);
    attitude_set_max_pitch(15.0f);

    xTaskCreate(control_task, "car_ctrl", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Car control initialized (cascade control with attitude)");
}

void car_control_update_params(const car_control_params_t *params) {
    if (params == NULL || s_params_mutex == NULL) return;

    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    s_params = *params;
    xSemaphoreGive(s_params_mutex);

    // 将新的 PID 参数实时更新到姿态控制模块
    attitude_set_speed_pid(params->speed_pid_kp, params->speed_pid_ki, params->speed_pid_kd);
    // 注意：转向外环的 P 增益可以通过 turn_gain 或者单独配置，这里将 turn_gain 作为转向增益系数
    // 为了灵活性，也可以直接使用 attitude_set_yaw_rate_pid 单独设置
    // 示例：将 turn_gain 映射为 yaw rate 外环的 KP（范围 0.2~1.5）
    float yaw_kp = params->turn_gain * 1.0f;
    if (yaw_kp < 0.1f) yaw_kp = 0.1f;
    if (yaw_kp > 2.0f) yaw_kp = 2.0f;
    attitude_set_yaw_rate_pid(yaw_kp, 0.0f, 0.0f);

    ESP_LOGI(TAG, "Params updated: speed=%.1f(0.01m/s), turn=%.1f, turn_gain=%.2f, PID_KP=%.2f, KI=%.2f, KD=%.2f",
             params->target_speed, params->target_turn, params->turn_gain,
             params->speed_pid_kp, params->speed_pid_ki, params->speed_pid_kd);
}

void car_control_get_params(car_control_params_t *params) {
    if (params == NULL) return;
    if (s_params_mutex == NULL) {
        memset(params, 0, sizeof(*params));
        return;
    }
    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    *params = s_params;
    xSemaphoreGive(s_params_mutex);
}