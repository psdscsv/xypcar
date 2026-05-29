#include "car_control.h"
#include "motor_control.h"
#include "attitude_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "CarCtrl";

// 共享数据
static car_control_params_t s_params = {
    .target_speed = 0,
    .target_turn = 0,
    .kp_roll = 5.0f,
    .kp_pitch = 3.0f,
    .speed_pitch_gain = 0.3f,
    .turn_gain = 1.0f,
};
static SemaphoreHandle_t s_params_mutex = NULL;

// 控制任务周期（ms）
#define CONTROL_PERIOD_MS 20

// 控制任务：周期调用 attitude_stabilize
static void control_task(void *pvParameters)
{
    car_control_params_t params;
    float left, right;
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        // 获取当前参数（含指令和增益）
        car_control_get_params(&params);

        // 应用转向增益到目标转向（此处做增益调整，因为 attitude_stabilize 接收的是调整后的转向）
        float adjusted_turn = params.target_turn * params.turn_gain;
        // 限幅到 [-100,100]
        if (adjusted_turn > 100.0f)
            adjusted_turn = 100.0f;
        if (adjusted_turn < -100.0f)
            adjusted_turn = -100.0f;

        // 姿态稳定控制（内部会使用 speed_pitch_gain 和 PID 参数）
        attitude_stabilize(params.target_speed, adjusted_turn, &left, &right);

        // 输出到电机
        if (!params.stop)
        {
            motor_set_speed(left, right);
        }
        else
        {
            motor_set_speed(0, 0); // 紧急停止
            attitude_clean_pid();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void car_control_init(void)
{
    s_params_mutex = xSemaphoreCreateMutex();
    if (s_params_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // 将初始参数设置到姿态控制模块
    attitude_set_roll_kp(s_params.kp_roll);
    attitude_set_pitch_kp(s_params.kp_pitch);
    attitude_set_speed_to_pitch_gain(s_params.speed_pitch_gain);

    // 创建控制任务
    xTaskCreate(control_task, "car_ctrl", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Car control module initialized");
}

void car_control_update_params(const car_control_params_t *params)
{
    if (params == NULL || s_params_mutex == NULL)
        return;

    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    s_params = *params;
    xSemaphoreGive(s_params_mutex);

    // 更新姿态控制器参数
    attitude_set_roll_kp(params->kp_roll);
    attitude_set_pitch_kp(params->kp_pitch);
    attitude_set_speed_to_pitch_gain(params->speed_pitch_gain);
    attitude_set_roll_turn_gain(params->roll_turn_gain);
    attitude_set_turn_speed_factor(params->turn_speed_factor);
    // turn_gain 在 car_control 的任务中使用，已乘入 adjusted_turn
}

void car_control_get_params(car_control_params_t *params)
{
    if (params == NULL)
        return;
    if (s_params_mutex == NULL)
    {
        memset(params, 0, sizeof(*params));
        return;
    }
    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    *params = s_params;
    xSemaphoreGive(s_params_mutex);
}