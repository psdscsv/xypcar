// attitude_control.c
#include "attitude_control.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "AttCtrl";

// 互补滤波参数
#define FILTER_ALPHA 0.98f

// 角度限幅
#define MAX_ROLL 30.0f
#define MAX_PITCH 30.0f

// 姿态内环 PID 默认参数
#define ROLL_P_DEFAULT 2.0f
#define ROLL_I_DEFAULT 0.3f
#define ROLL_D_DEFAULT 0.8f
#define PITCH_P_DEFAULT 2.5f
#define PITCH_I_DEFAULT 0.2f
#define PITCH_D_DEFAULT 0.6f

// 速度外环 PID 默认参数（控制期望俯仰角）
#define SPEED_KP_DEFAULT 1.8f
#define SPEED_KI_DEFAULT 0.4f
#define SPEED_KD_DEFAULT 0.1f

// 最大期望俯仰角（度），限制最大加速度
#define MAX_PITCH_CMD 50.0f

static float roll_angle = 0.0f;
static float pitch_angle = 0.0f;

static uint32_t last_time_ms = 0;

// PID 结构体
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID_t;

static PID_t pid_roll = {ROLL_P_DEFAULT, ROLL_I_DEFAULT, ROLL_D_DEFAULT, 0.0f, 0.0f};
static PID_t pid_pitch = {PITCH_P_DEFAULT, PITCH_I_DEFAULT, PITCH_D_DEFAULT, 0.0f, 0.0f};

// 速度外环 PID（输出期望俯仰角）
static PID_t pid_speed = {SPEED_KP_DEFAULT, SPEED_KI_DEFAULT, SPEED_KD_DEFAULT, 0.0f, 0.0f};

// 转向->期望滚转增益（度/100%转向）
static float roll_turn_gain = 0.6f;

static void pid_init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_update(PID_t *pid, float setpoint, float measurement, float dt, bool reset_integral)
{
    float error = setpoint - measurement;
    if (reset_integral)
    {
        pid->integral = 0.0f;
    }
    else
    {
        pid->integral += error * dt;
        // 积分限幅，防止饱和
        if (pid->integral > 100.0f)
            pid->integral = 100.0f;
        if (pid->integral < -100.0f)
            pid->integral = -100.0f;
    }
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

// 从加速度计计算俯仰和滚转（用于互补滤波）
static void accel_to_angles(float ax, float ay, float az, float *roll_acc, float *pitch_acc)
{
    *roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    *pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

static void attitude_update(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f)
    {
        last_time_ms = now_ms;
        return;
    }
    last_time_ms = now_ms;

    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);

    // 加速度计角度
    float roll_acc, pitch_acc;
    accel_to_angles(ax, ay, az, &roll_acc, &pitch_acc);

    // 陀螺仪积分（基于上一时刻滤波角度）
    float gyro_roll_new = roll_angle + gx * dt;
    float gyro_pitch_new = pitch_angle + gy * dt;

    // 互补滤波
    roll_angle = FILTER_ALPHA * gyro_roll_new + (1.0f - FILTER_ALPHA) * roll_acc;
    pitch_angle = FILTER_ALPHA * gyro_pitch_new + (1.0f - FILTER_ALPHA) * pitch_acc;

    // 限幅
    if (roll_angle > MAX_ROLL)
        roll_angle = MAX_ROLL;
    if (roll_angle < -MAX_ROLL)
        roll_angle = -MAX_ROLL;
    if (pitch_angle > MAX_PITCH)
        pitch_angle = MAX_PITCH;
    if (pitch_angle < -MAX_PITCH)
        pitch_angle = -MAX_PITCH;
}

void attitude_init(void)
{
    mpu6050_calibrate_gyro();
    vTaskDelay(pdMS_TO_TICKS(50));

    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);
    accel_to_angles(ax, ay, az, &roll_angle, &pitch_angle);
    last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Attitude init: roll=%.2f, pitch=%.2f", roll_angle, pitch_angle);
}

void attitude_get_angles(float *roll, float *pitch)
{
    *roll = roll_angle;
    *pitch = pitch_angle;
}

// 原始姿态稳定（无速度反馈，仅内环） – 兼容旧代码，不建议使用
void attitude_stabilize(float target_speed, float target_turn,
                        float *left_out, float *right_out)
{
    attitude_update();

    float pitch_setpoint = target_speed * 0.2f; // 简单映射
    if (pitch_setpoint > MAX_PITCH_CMD)
        pitch_setpoint = MAX_PITCH_CMD;
    if (pitch_setpoint < -MAX_PITCH_CMD)
        pitch_setpoint = -MAX_PITCH_CMD;
    float roll_setpoint = target_turn * roll_turn_gain;
    if (roll_setpoint > MAX_ROLL)
        roll_setpoint = MAX_ROLL;
    if (roll_setpoint < -MAX_ROLL)
        roll_setpoint = -MAX_ROLL;

    static uint32_t last_pid_ms = 0;
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - last_pid_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.02f;
    last_pid_ms = now_ms;

    bool reset_int = (fabsf(target_speed) < 0.1f && fabsf(target_turn) < 0.1f);
    float roll_corr = pid_update(&pid_roll, roll_setpoint, roll_angle, dt, reset_int);
    float pitch_corr = pid_update(&pid_pitch, pitch_setpoint, pitch_angle, dt, reset_int);

    const float MAX_CORR = 50.0f;
    roll_corr = fmaxf(-MAX_CORR, fminf(MAX_CORR, roll_corr));
    pitch_corr = fmaxf(-MAX_CORR, fminf(MAX_CORR, pitch_corr));

    // 混控：转向差速 + 滚转修正 + 俯仰修正（俯仰修正同向用于加减速）
    float left = target_turn - roll_corr + pitch_corr;
    float right = -target_turn + roll_corr + pitch_corr;

    *left_out = fmaxf(-100.0f, fminf(100.0f, left));
    *right_out = fmaxf(-100.0f, fminf(100.0f, right));
}

// 级联控制（速度外环 + 姿态内环） – 推荐使用
void attitude_stabilize_with_speed(float target_speed, float target_turn,
                                   float current_left_speed, float current_right_speed,
                                   float *left_out, float *right_out)
{
    attitude_update();

    // 当前实际线速度（取平均）
    float current_linear = (current_left_speed + current_right_speed) / 2.0f;

    // 1. 速度外环：计算期望俯仰角
    static uint32_t last_speed_ms = 0;
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - last_speed_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.02f;
    last_speed_ms = now_ms;

    bool reset_speed = (fabsf(target_speed) < 0.05f);
    float pitch_setpoint = pid_update(&pid_speed, target_speed, current_linear, dt, reset_speed);
    // 限制期望俯仰角
    if (pitch_setpoint > MAX_PITCH_CMD)
        pitch_setpoint = MAX_PITCH_CMD;
    if (pitch_setpoint < -MAX_PITCH_CMD)
        pitch_setpoint = -MAX_PITCH_CMD;

    // 2. 转向->期望滚转角
    float roll_setpoint = target_turn * roll_turn_gain;
    if (roll_setpoint > MAX_ROLL)
        roll_setpoint = MAX_ROLL;
    if (roll_setpoint < -MAX_ROLL)
        roll_setpoint = -MAX_ROLL;

    // 3. 姿态内环 PID（控制实际俯仰/滚转跟随期望）
    static uint32_t last_inner_ms = 0;
    now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    dt = (now_ms - last_inner_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.02f;
    last_inner_ms = now_ms;

    bool reset_inner = reset_speed; // 速度指令为零时重置内环积分
    float roll_corr = pid_update(&pid_roll, roll_setpoint, roll_angle, dt, reset_inner);
    float pitch_corr = pid_update(&pid_pitch, pitch_setpoint, pitch_angle, dt, reset_inner);

    const float MAX_CORR = 60.0f;
    roll_corr = fmaxf(-MAX_CORR, fminf(MAX_CORR, roll_corr));
    pitch_corr = fmaxf(-MAX_CORR, fminf(MAX_CORR, pitch_corr));

    // 4. 混控（最终电机指令）
    // 转向差速 + 滚转修正 + 俯仰修正（同向）
    float left = target_turn - roll_corr + pitch_corr;
    float right = -target_turn + roll_corr + pitch_corr;

    // 5. 安全保护：如果俯仰角过大且目标速度方向与倾斜方向一致，强制降低输出
    if (pitch_angle > 15.0f && target_speed > 0)
    {
        left *= 0.5f;
        right *= 0.5f;
    }
    else if (pitch_angle < -15.0f && target_speed < 0)
    {
        left *= 0.5f;
        right *= 0.5f;
    }

    *left_out = fmaxf(-100.0f, fminf(100.0f, left));
    *right_out = fmaxf(-100.0f, fminf(100.0f, right));

    // 可选：打印调试信息
    // ESP_LOGI(TAG, "target=%.2f, curr=%.2f, pitch_sp=%.1f, pitch=%.1f, left=%.0f, right=%.0f",
    //          target_speed, current_linear, pitch_setpoint, pitch_angle, *left_out, *right_out);
}

// ========== 参数设置接口 ==========
void attitude_set_roll_kp(float kp)
{
    pid_roll.kp = kp;
    ESP_LOGI(TAG, "Roll KP=%.2f", kp);
}
void attitude_set_pitch_kp(float kp)
{
    pid_pitch.kp = kp;
    ESP_LOGI(TAG, "Pitch KP=%.2f", kp);
}
void attitude_set_roll_kd(float kd)
{
    pid_roll.kd = kd;
    ESP_LOGI(TAG, "Roll KD=%.2f", kd);
}
void attitude_set_pitch_kd(float kd)
{
    pid_pitch.kd = kd;
    ESP_LOGI(TAG, "Pitch KD=%.2f", kd);
}

void attitude_set_speed_pid(float kp, float ki, float kd)
{
    pid_speed.kp = kp;
    pid_speed.ki = ki;
    pid_speed.kd = kd;
    ESP_LOGI(TAG, "Speed PID: KP=%.2f, KI=%.2f, KD=%.2f", kp, ki, kd);
}

void attitude_set_speed_to_pitch_gain(float gain)
{
    // 旧接口，已弃用，留空
    ESP_LOGW(TAG, "speed_to_pitch_gain is deprecated, use speed PID instead");
}

void attitude_clean_pid(void)
{
    pid_roll.integral = 0.0f;
    pid_roll.prev_error = 0.0f;
    pid_pitch.integral = 0.0f;
    pid_pitch.prev_error = 0.0f;
    pid_speed.integral = 0.0f;
    pid_speed.prev_error = 0.0f;
}