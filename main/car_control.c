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
#include <math.h>
#include "mpu6050.h"

static const char *TAG = "CarCtrl";

/* 默认速度环 PI：输入 m/s 误差，输出俯仰角（度）
 * 直观标定：0.5 m/s 误差 → 约 30° 俯仰角 ⇒ kp ≈ 60 */
#define CAR_SPEED_KP_DEFAULT  60.0f
#define CAR_SPEED_KI_DEFAULT  10.0f
#define CAR_SPEED_KD_DEFAULT   0.0f

static car_control_params_t s_params = {
    .stop           = 1,
    .target_speed   = 0.0f,          /* m/s */
    .target_turn    = 0.0f,          /* °/s */
    .turn_gain      = 1.0f,
    .speed_pid_kp   = CAR_SPEED_KP_DEFAULT,
    .speed_pid_ki   = CAR_SPEED_KI_DEFAULT,
    .speed_pid_kd   = CAR_SPEED_KD_DEFAULT,
};

static SemaphoreHandle_t s_params_mutex = NULL;

#define CONTROL_PERIOD_MS 20   /* 50Hz */

static void control_task(void *pvParameters) {
    car_control_params_t params;
    float left_spd, right_spd;              /* m/s */
    float target_linear_spd;                /* m/s */
    float target_angular_rate_dps;          /* °/s */
    float left_out, right_out;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        car_control_get_params(&params);

        if (params.stop) {
            motor_set_speed(0, 0);
            attitude_clean_pid();
            calibrate_zero_offset();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        /* 获取当前轮速（m/s） */
        encoder_get_speed(&left_spd, &right_spd);

        /* 单位已经是 m/s，直接使用 */
        target_linear_spd       = params.target_speed;
        target_angular_rate_dps = params.target_turn;

        /* 级联控制：速度环(PI) + 姿态内环(PD) + 转向环(P) */
        attitude_stabilize_with_speed(target_linear_spd, target_angular_rate_dps,
                                      left_spd, right_spd,
                                      &left_out, &right_out);

        motor_set_speed(left_out, right_out);

        ESP_LOGD(TAG,
                 "tgt: v=%.2f m/s, w=%.2f °/s | meas: L=%.2f R=%.2f | out: L=%.2f R=%.2f",
                 target_linear_spd, target_angular_rate_dps,
                 left_spd, right_spd, left_out, right_out);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void car_control_init(void) {
    s_params_mutex = xSemaphoreCreateMutex();
    if (s_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    /* 把默认速度环 PI 同步给姿态控制器（flag=1 表示速度环） */
    attitude_set_pid(1,
                     s_params.speed_pid_kp,
                     s_params.speed_pid_ki,
                     s_params.speed_pid_kd);
    attitude_set_max_pitch(45.0f);

    xTaskCreate(control_task, "car_ctrl", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Car control initialized. Speed unit: m/s, max=%.2f m/s",
             CAR_MAX_SPEED_MS);
}

void car_control_update_params(const car_control_params_t *params) {
    if (params == NULL || s_params_mutex == NULL) return;

    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    s_params = *params;
    xSemaphoreGive(s_params_mutex);

    /* 更新速度环 PID（flag=1） */
    attitude_set_pid(1,
                     params->speed_pid_kp,
                     params->speed_pid_ki,
                     params->speed_pid_kd);

    ESP_LOGD(TAG,
             "Params: v=%.2f m/s, w=%.2f °/s, speed_PID=(%.2f,%.2f,%.2f)",
             params->target_speed, params->target_turn,
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