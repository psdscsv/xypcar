// attitude_control.c
#include "attitude_control.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "AttCtrl";

// 互补滤波参数
#define FILTER_ALPHA 0.98f // 加速度计权重小，陀螺仪权重大

// PID 参数（需根据实际调校）
#define ROLL_P 5.0f // 侧倾比例系数
#define ROLL_I 0.5f
#define ROLL_D 1.0f
#define PITCH_P 3.0f // 俯仰比例系数（前后稳定）
#define PITCH_I 0.2f
#define PITCH_D 0.8f

// 角度限幅
#define MAX_ROLL 30.0f
#define MAX_PITCH 30.0f

static float roll_angle = 0.0f;  // 滤波后的滚转角（度）
static float pitch_angle = 0.0f; // 滤波后的俯仰角（度）

// 陀螺仪积分用
static float gyro_roll = 0.0f;
static float gyro_pitch = 0.0f;
static uint32_t last_time_ms = 0;

// PID 控制器状态
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID_t;

static PID_t pid_roll = {ROLL_P, ROLL_I, ROLL_D, 0.0f, 0.0f};
static PID_t pid_pitch = {PITCH_P, PITCH_I, PITCH_D, 0.0f, 0.0f};

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
    // 根据用户描述：右侧倾斜时 ay 减少（负），向前倾斜时 ax 增加（正）
    // 计算 roll = atan2(ay, az)  注意符号：右手系，Y 向右为正
    *roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    // 计算 pitch = atan2(-ax, sqrt(ay*ay+az*az))
    *pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

void attitude_init(void)
{
    // 校准陀螺仪（必须静止）
    mpu6050_calibrate_gyro();
    vTaskDelay(pdMS_TO_TICKS(50));

    // 读取初始角度
    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);
    accel_to_angles(ax, ay, az, &roll_angle, &pitch_angle);
    gyro_roll = roll_angle;
    gyro_pitch = pitch_angle;
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

    // 读取 MPU6050
    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);

    // 陀螺仪积分（注意单位：度/秒）
    gyro_roll += gx * dt;
    gyro_pitch += gy * dt;

    // 加速度计计算角度
    float roll_acc, pitch_acc;
    accel_to_angles(ax, ay, az, &roll_acc, &pitch_acc);

    // 互补滤波
    roll_angle = FILTER_ALPHA * gyro_roll + (1.0f - FILTER_ALPHA) * roll_acc;
    pitch_angle = FILTER_ALPHA * gyro_pitch + (1.0f - FILTER_ALPHA) * pitch_acc;

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

void attitude_stabilize(float target_speed, float target_turn,
                        float *left_out, float *right_out)
{
    // 更新姿态（先获取最新数据）
    attitude_update();

    // 目标姿态：我们希望车体保持水平（roll=0, pitch=0）
    float roll_setpoint = 0.0f;
    float pitch_setpoint = 0.0f;

    // PID 计算修正量（修正量范围限制在 -50 ~ 50，避免过度补偿）
    float dt = 0.02f; // 假设调用周期 20ms（实际由主循环频率决定）
    bool reset_int = (fabs(target_speed) < 0.1f && fabs(target_turn) < 0.1f);
    float roll_correction = pid_update(&pid_roll, roll_setpoint, roll_angle, dt, reset_int);
    float pitch_correction = pid_update(&pid_pitch, pitch_setpoint, pitch_angle, dt, reset_int);

    // 限制修正量范围
    if (roll_correction > 50.0f)
        roll_correction = 50.0f;
    if (roll_correction < -50.0f)
        roll_correction = -50.0f;
    if (pitch_correction > 50.0f)
        pitch_correction = 50.0f;
    if (pitch_correction < -50.0f)
        pitch_correction = -50.0f;

    // 混控：左轮 = 速度 + 转向 - 侧倾修正 + 俯仰修正（俯仰修正可辅助前后稳定性）
    // 这里侧倾修正直接作用于差速：向右倾（正roll）则减小左轮、增大右轮，使之扶正
    // 俯仰修正：前倾（正pitch）时，应减速或后退补偿，这里简单减速度
    float left = target_speed + target_turn - roll_correction - pitch_correction * 0.5f;
    float right = target_speed - target_turn + roll_correction - pitch_correction * 0.5f;

    // 限制输出范围
    if (left > 100.0f)
        left = 100.0f;
    if (left < -100.0f)
        left = -100.0f;
    if (right > 100.0f)
        right = 100.0f;
    if (right < -100.0f)
        right = -100.0f;

    *left_out = left;
    *right_out = right;

    // 可打印调试信息
    // ESP_LOGI(TAG, "roll=%.1f, pitch=%.1f, corrR=%.1f, corrP=%.1f -> L=%.0f R=%.0f",
    //          roll_angle, pitch_angle, roll_correction, pitch_correction, left, right);
}
void attitude_set_roll_kp(float kp)
{
    pid_roll.kp = kp;
}