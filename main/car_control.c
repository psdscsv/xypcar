// car_control.c
#include "car_control.h"
#include "motor_control.h"
#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

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

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} SpeedPID_t;

static SpeedPID_t s_speed_pid = {0};
#define CONTROL_PERIOD_MS 20

static float speed_pid_update(SpeedPID_t *pid, float setpoint, float measurement, float dt, bool reset) {
    float error = setpoint - measurement;
    if (reset) {
        pid->integral = 0.0f;
    } else {
        pid->integral += error * dt;
        if (pid->integral > 100.0f) pid->integral = 100.0f;
        if (pid->integral < -100.0f) pid->integral = -100.0f;
    }
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    return output;
}

static void control_task(void *pvParameters) {
    car_control_params_t params;
    float left_speed, right_speed;
    float target_linear_speed_ms;
    float base_output, left_out, right_out;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_time_ms = 0;

    while (1) {
        car_control_get_params(&params);

        if (params.stop) {
            motor_set_speed(0, 0);
            s_speed_pid.integral = 0;
            s_speed_pid.prev_error = 0;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        encoder_get_speed(&left_speed, &right_speed);
        float current_linear = (left_speed + right_speed) * 0.5f;
        target_linear_speed_ms = params.target_speed * 0.01f;

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now_ms - last_time_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 0.05f) dt = 0.02f;
        last_time_ms = now_ms;

        bool reset_int = (fabsf(params.target_speed) < 1.0f && fabsf(params.target_turn) < 1.0f);
        base_output = speed_pid_update(&s_speed_pid, target_linear_speed_ms, current_linear, dt, reset_int);

        float diff = params.target_turn * params.turn_gain;
        if (diff > 100.0f) diff = 100.0f;
        if (diff < -100.0f) diff = -100.0f;

        left_out = base_output - diff;
        right_out = base_output + diff;

        if (left_out > 100.0f) left_out = 100.0f;
        if (left_out < -100.0f) left_out = -100.0f;
        if (right_out > 100.0f) right_out = 100.0f;
        if (right_out < -100.0f) right_out = -100.0f;

        motor_set_speed(left_out, right_out);

        ESP_LOGD(TAG, "Tgt=%.2fm/s Cur=%.2f Base=%.1f L=%.1f R=%.1f",
                 target_linear_speed_ms, current_linear, base_output, left_out, right_out);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void car_control_init(void) {
    s_params_mutex = xSemaphoreCreateMutex();
    if (s_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    s_speed_pid.kp = s_params.speed_pid_kp;
    s_speed_pid.ki = s_params.speed_pid_ki;
    s_speed_pid.kd = s_params.speed_pid_kd;
    s_speed_pid.integral = 0;
    s_speed_pid.prev_error = 0;

    xTaskCreate(control_task, "car_ctrl", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Car control initialized (pure speed control)");
}

void car_control_update_params(const car_control_params_t *params) {
    if (params == NULL || s_params_mutex == NULL) return;

    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    s_params = *params;
    xSemaphoreGive(s_params_mutex);

    // 更新 PID 运行参数
    s_speed_pid.kp = params->speed_pid_kp;
    s_speed_pid.ki = params->speed_pid_ki;
    s_speed_pid.kd = params->speed_pid_kd;

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