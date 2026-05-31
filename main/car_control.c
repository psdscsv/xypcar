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

static car_control_params_t s_params = {
    .stop = 1,
    .target_speed = 0,
    .target_turn = 0,
    .turn_gain = 0.5f,
    .speed_pid_kp = 0.01f,     // 调整为适合 PPS 单位的初始值
    .speed_pid_ki = 0.001f,
    .speed_pid_kd = 0.0f,
};

static SemaphoreHandle_t s_params_mutex = NULL;

#define CONTROL_PERIOD_MS 20   // 50Hz 控制周期

// 零位校准相关
#define ZERO_CALIB_STABLE_THRESHOLD 0.5f   // 角度变化小于0.5度认为稳定
#define ZERO_CALIB_SAMPLE_COUNT   50       // 需要连续稳定多少次
#define ZERO_CALIB_SAMPLE_INTERVAL_MS 20   // 每次采样间隔(ms)

static void calibrate_zero_offset(void) {
    float roll_sum = 0, pitch_sum = 0;
    int sample_count = 0;
    float last_roll = 0, last_pitch = 0;
    int stable_counter = 0;
    bool first_sample = true;

    ESP_LOGI(TAG, "Starting zero offset calibration... Keep robot still!");

    float ax, ay, az, gx, gy, gz;
    float roll_angle = 0, pitch_angle = 0;
    uint32_t last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    const float alpha = 0.96f;

    while (stable_counter < ZERO_CALIB_SAMPLE_COUNT) {
        mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0.0f || dt > 0.05f) dt = 0.02f;
        last_time = now;

        float roll_acc = atan2f(ay, az) * 180.0f / M_PI;
        float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

        float gyro_roll = roll_angle + gx * dt;
        float gyro_pitch = pitch_angle + gy * dt;

        roll_angle = alpha * gyro_roll + (1 - alpha) * roll_acc;
        pitch_angle = alpha * gyro_pitch + (1 - alpha) * pitch_acc;

        if (!first_sample) {
            float roll_change = fabsf(roll_angle - last_roll);
            float pitch_change = fabsf(pitch_angle - last_pitch);
            if (roll_change < ZERO_CALIB_STABLE_THRESHOLD && pitch_change < ZERO_CALIB_STABLE_THRESHOLD) {
                stable_counter++;
                roll_sum += roll_angle;
                pitch_sum += pitch_angle;
                sample_count++;
            } else {
                stable_counter = 0;
                roll_sum = 0;
                pitch_sum = 0;
                sample_count = 0;
            }
        }
        last_roll = roll_angle;
        last_pitch = pitch_angle;
        first_sample = false;

        vTaskDelay(pdMS_TO_TICKS(ZERO_CALIB_SAMPLE_INTERVAL_MS));
    }

    if (sample_count > 0) {
        float roll_offset = roll_sum / sample_count;
        float pitch_offset = pitch_sum / sample_count;
        attitude_set_zero_offset(roll_offset, pitch_offset);
        ESP_LOGI(TAG, "Zero offset calibration complete: roll=%.2f deg, pitch=%.2f deg", roll_offset, pitch_offset);
    } else {
        ESP_LOGW(TAG, "Zero offset calibration failed, no stable samples");
    }
}

static void control_task(void *pvParameters) {
    car_control_params_t params;
    float left_pps, right_pps;                 // 单位：脉冲/秒
    float target_linear_pps;                  // 目标线速度，单位：脉冲/秒
    float target_angular_rate_dps;            // 单位：°/s
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

        // 获取当前轮速（脉冲/秒）
        // 注意：需要 encoder_get_speed_pps 函数,现在还是使用原来的函数获取线速度
        encoder_get_speed(&left_pps, &right_pps);

        // 目标速度直接使用 BLE 发送的 target_speed（单位：脉冲/秒）
        target_linear_pps = params.target_speed;
        target_angular_rate_dps = params.target_turn;   // 仍为 °/s

        // 调用级联控制核心（速度外环将使用脉冲/秒作为单位）
        attitude_stabilize_with_speed(target_linear_pps, target_angular_rate_dps,
                                      left_pps, right_pps,
                                      &left_out, &right_out);

        motor_set_speed(left_out, right_out);

        ESP_LOGD(TAG, "tgt_pps=%.1f, cur_pps=%.1f, turn=%.1f, left=%.1f%%, right=%.1f%%",
                 target_linear_pps, (left_pps + right_pps) * 0.5f,
                 target_angular_rate_dps, left_out, right_out);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void car_control_init(void) {
    s_params_mutex = xSemaphoreCreateMutex();
    if (s_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // 将 BLE 传入的 PID 参数同步到姿态控制模块（注意量纲已变，需要重新整定）
    attitude_set_speed_pid(s_params.speed_pid_kp, s_params.speed_pid_ki, s_params.speed_pid_kd);
    attitude_set_yaw_rate_pid(0.5f, 0.0f, 0.0f);
    attitude_set_max_pitch(45.0f);

    xTaskCreate(control_task, "car_ctrl", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Car control initialized (speed unit: pulses/sec)");
}

void car_control_update_params(const car_control_params_t *params) {
    if (params == NULL || s_params_mutex == NULL) return;

    xSemaphoreTake(s_params_mutex, portMAX_DELAY);
    s_params = *params;
    xSemaphoreGive(s_params_mutex);

    attitude_set_speed_pid(params->speed_pid_kp, params->speed_pid_ki, params->speed_pid_kd);
    float yaw_kp = params->turn_gain * 1.0f;
    if (yaw_kp < 0.1f) yaw_kp = 0.1f;
    if (yaw_kp > 2.0f) yaw_kp = 2.0f;
    attitude_set_yaw_rate_pid(yaw_kp, 0.0f, 0.0f);

    ESP_LOGI(TAG, "Params updated: speed=%.1f pps, turn=%.1f, turn_gain=%.2f, PID_KP=%.4f, KI=%.4f, KD=%.2f",
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