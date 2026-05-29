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

// PID 默认参数（可通过 BLE 动态修改）
#define ROLL_P_DEFAULT 5.0f
#define ROLL_I_DEFAULT 0.5f
#define ROLL_D_DEFAULT 1.0f
#define PITCH_P_DEFAULT 3.0f
#define PITCH_I_DEFAULT 0.2f
#define PITCH_D_DEFAULT 0.8f

// 角度限幅
#define MAX_ROLL 30.0f
#define MAX_PITCH 30.0f

static float roll_angle = 0.0f;
static float pitch_angle = 0.0f;

// 陀螺仪积分用
static uint32_t last_time_ms = 0;
static float roll_turn_gain = 0.8f;    // 转向指令到期望滚转角增益 (度/100%转向)
static float turn_speed_factor = 0.5f; // 高速转向衰减系数: 实际转向 = 目标转向 * (1 - |speed|/100 * factor)
// 输出速率限制 (度/秒)
static float max_accel = 150.0f; // 最大加减速率 (%/s)
static float last_left_out = 0.0f, last_right_out = 0.0f;
// PID 控制器状态
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

// 外环参数：速度指令 → 期望俯仰角增益（度/100%速度）
static float speed_to_pitch_gain = 0.3f; // 默认 0.3，可调

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
        // 积分限幅
        if (pid->integral > 100.0f)
            pid->integral = 100.0f;
        if (pid->integral < -100.0f)
            pid->integral = -100.0f;
    }
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

/**
 * @brief 从加速度计计算俯仰和滚转（仅用于互补滤波中的加速度修正）
 */
static void accel_to_angles(float ax, float ay, float az, float *roll_acc, float *pitch_acc)
{
    *roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    *pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

void attitude_init(void)
{
    mpu6050_calibrate_gyro();
    vTaskDelay(pdMS_TO_TICKS(50));

    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);
    accel_to_angles(ax, ay, az, &roll_angle, &pitch_angle);
    last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Initial roll=%.2f, pitch=%.2f", roll_angle, pitch_angle);
}

void attitude_get_angles(float *roll, float *pitch)
{
    *roll = roll_angle;
    *pitch = pitch_angle;
}

/**
 * @brief 周期性更新姿态（建议在 10~20ms 循环中调用）
 */
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

    // 加速度计计算角度
    float roll_acc, pitch_acc;
    accel_to_angles(ax, ay, az, &roll_acc, &pitch_acc);

    // 正确积分：以上一时刻滤波角度为基础，加上陀螺仪增量
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

    // ESP_LOGI(TAG, "Roll=%.2f, Pitch=%.2f", roll_angle, pitch_angle);
}

void attitude_stabilize(float target_speed, float target_turn,
                        float *left_out, float *right_out)
{
    attitude_update();

    // ========== 1. 速度‑转向增益调度 ==========
    float speed_abs = fabsf(target_speed);
    float turn_factor = 1.0f - (speed_abs / 100.0f) * turn_speed_factor;
    if (turn_factor < 0.2f)
        turn_factor = 0.2f; // 最低保留20%灵敏度
    float adjusted_turn = target_turn * turn_factor;

    // ========== 2. 外环：速度 → 期望俯仰角 ==========
    float pitch_setpoint = target_speed * speed_to_pitch_gain;
    if (pitch_setpoint > MAX_PITCH)
        pitch_setpoint = MAX_PITCH;
    if (pitch_setpoint < -MAX_PITCH)
        pitch_setpoint = -MAX_PITCH;

    // ========== 3. 动态期望滚转角（转向时允许车身倾斜） ==========
    float roll_setpoint = adjusted_turn * roll_turn_gain;
    if (roll_setpoint > MAX_ROLL)
        roll_setpoint = MAX_ROLL;
    if (roll_setpoint < -MAX_ROLL)
        roll_setpoint = -MAX_ROLL;

    // ========== 4. PID 计算 ==========
    static uint32_t last_pid_ms = 0;
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - last_pid_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.02f;
    last_pid_ms = now_ms;

    bool reset_int = (fabsf(target_speed) < 0.1f && fabsf(target_turn) < 0.1f);
    float roll_correction = pid_update(&pid_roll, roll_setpoint, roll_angle, dt, reset_int);
    float pitch_correction = pid_update(&pid_pitch, pitch_setpoint, pitch_angle, dt, reset_int);

    // 修正量限幅
    const float MAX_CORR = 50.0f;
    roll_correction = fmaxf(-MAX_CORR, fminf(MAX_CORR, roll_correction));
    pitch_correction = fmaxf(-MAX_CORR, fminf(MAX_CORR, pitch_correction));

    // ========== 5. 混控（注意转向已经过调度，不再重复乘 turn_gain） ==========
    float left = target_speed + adjusted_turn - roll_correction + pitch_correction;
    float right = target_speed - adjusted_turn + roll_correction + pitch_correction;

    // ========== 6. 输出速率限制（平滑加减速） ==========
    float dt_sec = dt > 0.01f ? dt : 0.02f;
    float max_change = max_accel * dt_sec;
    left = fmaxf(last_left_out - max_change, fminf(last_left_out + max_change, left));
    right = fmaxf(last_right_out - max_change, fminf(last_right_out + max_change, right));
    last_left_out = left;
    last_right_out = right;

    // 最终限幅
    left = fmaxf(-100.0f, fminf(100.0f, left));
    right = fmaxf(-100.0f, fminf(100.0f, right));

    *left_out = left;
    *right_out = right;
}
void attitude_set_roll_kp(float kp)
{
    pid_roll.kp = kp;
    ESP_LOGI(TAG, "Roll KP set to %.2f", kp);
}

void attitude_set_pitch_kp(float kp)
{
    pid_pitch.kp = kp;
    ESP_LOGI(TAG, "Pitch KP set to %.2f", kp);
}

void attitude_set_speed_to_pitch_gain(float gain)
{
    if (gain >= 0.0f && gain <= 1.0f)
    {
        speed_to_pitch_gain = gain;
        ESP_LOGI(TAG, "Speed->Pitch gain set to %.2f", gain);
    }
    else
    {
        ESP_LOGW(TAG, "Invalid gain %.2f, must be [0,1]", gain);
    }
}
void attitude_clean_pid(void)
{
    pid_roll.integral = 0.0f;
    pid_roll.prev_error = 0.0f;
    pid_pitch.integral = 0.0f;
    pid_pitch.prev_error = 0.0f;
}
void attitude_set_roll_turn_gain(float gain)
{
    if (gain >= 0.0f && gain <= 2.0f)
    {
        roll_turn_gain = gain;
        ESP_LOGI(TAG, "Roll turn gain set to %.2f", gain);
    }
}

void attitude_set_turn_speed_factor(float factor)
{
    if (factor >= 0.0f && factor <= 1.0f)
    {
        turn_speed_factor = factor;
        ESP_LOGI(TAG, "Turn speed factor set to %.2f", factor);
    }
}