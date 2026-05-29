// car_control.c
#include "car_control.h"
#include "motor_control.h"
#include "attitude_control.h"
#include "encoder.h"
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
    .kp_roll = 2.0f,
    .kp_pitch = 2.5f,
    .speed_pitch_gain = 0.15f, // 不再使用
    .turn_gain = 0.5f,
    .max_linear_speed = 2.0f, // 最大线速度 m/s（需标定）
    .speed_pid_kp = 1.8f,
    .speed_pid_ki = 0.4f,
    .speed_pid_kd = 0.1f,
};

static SemaphoreHandle_t s_params_mutex = NULL;

#define CONTROL_PERIOD_MS 20 // 20ms 控制周期

// 控制任务
static void control_task(void *pvParameters)
{
    car_control_params_t params;
    float left_out, right_out;
    float left_speed, right_speed;
    float target_speed_ms; // 目标线速度 (m/s)
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        // 获取当前控制参数（含目标速度和转向）
        car_control_get_params(&params);

        if (params.stop)
        {
            motor_set_speed(0, 0);
            attitude_clean_pid();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        // 读取编码器实际速度
        encoder_get_speed(&left_speed, &right_speed);

        // 将目标速度（-100~100）映射为线速度 (m/s)
        target_speed_ms = params.target_speed / 100.0f * params.max_linear_speed;

        // 应用转向增益（灵敏度调节）
        float target_turn = params.target_turn * params.turn_gain;
        if (target_turn > 100.0f)
            target_turn = 100.0f;
        if (target_turn < -100.0f)
            target_turn = -100.0f;
        target_speed_ms = params.target_speed / 100.0f * params.max_linear_speed;
        float target_angular_rate = params.target_turn / 100.0f * params.max_angular_speed; // 新增
                                                                                            // 姿态与速度级联控制
        attitude_stabilize_with_speed(target_speed_ms, target_angular_rate,
                                      left_speed, right_speed,
                                      &left_out, &right_out);

        // 输出到电机
        motor_set_speed(left_out, right_out);

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
    s_params.max_angular_speed = 30.0f; // 对应100%目标转向的角速度(°/s)，需根据实际情况调整
    s_params.stop = 1;
    // 将初始 PID 参数设置到姿态控制模块
    attitude_set_roll_kp(s_params.kp_roll);
    attitude_set_pitch_kp(s_params.kp_pitch);
    attitude_set_speed_pid(s_params.speed_pid_kp, s_params.speed_pid_ki, s_params.speed_pid_kd);

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

    // 更新姿态控制模块中的动态参数
    attitude_set_roll_kp(params->kp_roll);
    attitude_set_pitch_kp(params->kp_pitch);
    attitude_set_speed_pid(params->speed_pid_kp, params->speed_pid_ki, params->speed_pid_kd);

    ESP_LOGD(TAG, "Params updated: spd=%.1f, turn=%.1f, kpR=%.2f, kpP=%.2f, spKp=%.2f, spKi=%.2f, maxSpd=%.2f",
             params->target_speed, params->target_turn,
             params->kp_roll, params->kp_pitch,
             params->speed_pid_kp, params->speed_pid_ki,
             params->max_linear_speed);
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