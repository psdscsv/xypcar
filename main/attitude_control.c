// attitude_control.c
#include "attitude_control.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "AttCtrl";

#define FILTER_ALPHA 0.96f
#define MAX_ROLL     30.0f   // 滚转角限幅（仅用于监控）
#define MAX_PITCH    30.0f   // 俯仰角限幅（仅用于监控）

// 姿态内环 PD 默认参数
#define PITCH_P_DEFAULT 2.0f
#define PITCH_D_DEFAULT 0.1f

// 线速度外环 PI 默认参数
#define SPEED_KP_DEFAULT 40.0f
#define SPEED_KI_DEFAULT 8.0f

// 偏航角速度外环 P 默认参数x
#define YAW_RATE_KP_DEFAULT 0.5f

// 最大期望俯仰角（度）
static float max_pitch_cmd = 45.0f;

static float roll_angle = 0.0f;
static float pitch_angle = 0.0f;
static float pitch_rate = 0.0f;   // 俯仰角速度 (°/s)
static float current_yaw_rate = 0.0f;

static uint32_t last_time_ms = 0;

static float roll_offset = 0.0f;
static float pitch_offset = 0.0f;
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID_t;

static PID_t pid_pitch = {PITCH_P_DEFAULT, 0.0f, PITCH_D_DEFAULT, 0.0f, 0.0f};
static PID_t pid_speed = {SPEED_KP_DEFAULT, SPEED_KI_DEFAULT,0.0f, 0.0f, 0.0f};
static PID_t pid_yaw_rate = {YAW_RATE_KP_DEFAULT, 0.0f, 0.0f, 0.0f, 0.0f};

static void pid_init(PID_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_update(PID_t *pid, float setpoint, float measurement, float dt, bool reset_integral) {
    float error = setpoint - measurement;
    if (reset_integral) {
        pid->integral = 0.0f;
    } else {
        pid->integral += error * dt;
        // 积分限幅
        if (pid->integral > 100.0f) pid->integral = 100.0f;
        if (pid->integral < -100.0f) pid->integral = -100.0f;
    }
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

// 从加速度计计算俯仰和滚转（用于互补滤波）
static void accel_to_angles(float ax, float ay, float az, float *roll_acc, float *pitch_acc) {
    *roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    *pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

// 姿态更新（互补滤波），同时计算俯仰角速度
static void attitude_update(void) {
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f) {
        last_time_ms = now_ms;
        return;
    }
    last_time_ms = now_ms;

    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);

    // 保存偏航角速度（已校准）
    current_yaw_rate = gz;

    // 加速度计角度
    float roll_acc, pitch_acc;
    accel_to_angles(ax, ay, az, &roll_acc, &pitch_acc);

    // 陀螺仪积分
    float gyro_roll_new  = roll_angle + gx * dt;
    float gyro_pitch_new = pitch_angle + gy * dt;

// 互补滤波
roll_angle  = FILTER_ALPHA * gyro_roll_new  + (1.0f - FILTER_ALPHA) * roll_acc;
pitch_angle = FILTER_ALPHA * gyro_pitch_new + (1.0f - FILTER_ALPHA) * pitch_acc;

// 俯仰角速度直接使用陀螺仪 Y 轴（已减过零偏，但陀螺仪零偏已在 MPU6050 校准中处理，这里不再减）
pitch_rate = gy;

    // 限幅
    if (roll_angle > MAX_ROLL)  roll_angle = MAX_ROLL;
    if (roll_angle < -MAX_ROLL) roll_angle = -MAX_ROLL;
    if (pitch_angle > MAX_PITCH) pitch_angle = MAX_PITCH;
    if (pitch_angle < -MAX_PITCH) pitch_angle = -MAX_PITCH;

}
void calibrate_zero_offset(void) {
// 零位校准相关
#define ZERO_CALIB_STABLE_THRESHOLD 0.5f   // 角度变化小于0.5度认为稳定
#define ZERO_CALIB_SAMPLE_COUNT   50       // 需要连续稳定多少次
#define ZERO_CALIB_SAMPLE_INTERVAL_MS 20   // 每次采样间隔(ms)    
    static float last_roll = 0.0f, last_pitch = 0.0f;
    static int stable_count = 0;
    attitude_update();
    if(fabsf(last_roll-roll_angle) > ZERO_CALIB_STABLE_THRESHOLD||fabsf(last_pitch-pitch_angle) > ZERO_CALIB_STABLE_THRESHOLD) {
        last_roll = roll_angle;
        last_pitch = pitch_angle;
        stable_count = 0; // 不稳定，重置计数
        //ESP_LOGI(TAG, "Not stable for zero calib: roll=%.2f, pitch=%.2f", roll_angle, pitch_angle);
    } else {
        stable_count++;
        if(stable_count >= ZERO_CALIB_SAMPLE_COUNT) {
            // 稳定足够次数，进行校准
            attitude_set_zero_offset(roll_angle, pitch_angle);
            //ESP_LOGI(TAG, "Zero calibrated: roll=%.2f, pitch=%.2f", roll_angle, pitch_angle);
            stable_count = 0; // 重置计数，等待下次校准
        }
    }
}
void attitude_set_zero_offset(float roll_off, float pitch_off) {
    roll_offset = roll_off;
    pitch_offset = pitch_off;
    ESP_LOGI(TAG, "Zero offset set: roll=%.2f, pitch=%.2f", roll_offset, pitch_offset);
}
void attitude_init(void) {
    mpu6050_calibrate_gyro();
    vTaskDelay(pdMS_TO_TICKS(50));
    float ax, ay, az, gx, gy, gz;
    mpu6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);
    accel_to_angles(ax, ay, az, &roll_angle, &pitch_angle);
    last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Attitude init: roll=%.2f, pitch=%.2f", roll_angle, pitch_angle);
}

void attitude_get_yaw_rate(float *yaw_rate) {
    *yaw_rate = current_yaw_rate;
}

// 核心级联控制函数
void attitude_stabilize_with_speed(float target_linear_speed, float target_angular_rate,
                                   float current_left_speed, float current_right_speed,
                                   float *left_out, float *right_out) {
    // 1. 更新姿态（互补滤波）
    attitude_update();
    float offset_roll = roll_angle - roll_offset;
    float offset_pitch = pitch_angle - pitch_offset;

    float current_linear = (current_left_speed + current_right_speed) * 0.5f;

    // 时间计算（使用静态变量记录上次调用时间）
    static uint32_t last_control_ms = 0;
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - last_control_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.02f;
    last_control_ms = now_ms;

    // 判断是否应重置积分（目标接近零且当前速度小）
    bool reset_integral = (fabsf(target_linear_speed) < 0.05f && fabsf(target_angular_rate) < 1.0f);

    // ========== 2. 速度外环（PI）→ 期望俯仰角 ==========
    float pitch_setpoint = pid_update(&pid_speed, target_linear_speed, current_linear, dt, reset_integral);
    
    // 限制期望俯仰角范围
    if (pitch_setpoint > max_pitch_cmd) pitch_setpoint = max_pitch_cmd;
    if (pitch_setpoint < -max_pitch_cmd) pitch_setpoint = -max_pitch_cmd;

    // ========== 3. 转向外环（P）→ 期望差速系数 ==========
    float diff_setpoint = pid_update(&pid_yaw_rate, target_angular_rate, current_yaw_rate, dt, reset_integral);
    // 差速系数限制在 [-100, 100]
    if (diff_setpoint > 100.0f) diff_setpoint = 100.0f;
    if (diff_setpoint < -100.0f) diff_setpoint = -100.0f;

    // ========== 4. 姿态内环（PD）→ 同向力矩 ==========
    // 注意：对于俯仰角，微分项使用负的角速度（阻尼）
    float pitch_error = pitch_setpoint - offset_pitch;//在这里进行0偏补偿
    float pitch_corr = pid_pitch.kp * pitch_error - pid_pitch.kd * pitch_rate;
    // 限幅
    const float MAX_PITCH_OUT = 100.0f;
    if (pitch_corr > MAX_PITCH_OUT) pitch_corr = MAX_PITCH_OUT;
    if (pitch_corr < -MAX_PITCH_OUT) pitch_corr = -MAX_PITCH_OUT;

    // ========== 5. 混控 ==========
    float left  = pitch_corr - diff_setpoint;
    float right = pitch_corr + diff_setpoint;

    // 安全保护：实际俯仰角过大时降低输出，防止侧翻
    if (fabsf(offset_pitch) > 40.0f) {
        //left  *= 0.5f;
        //right *= 0.5f;
    }
    // 滚转角过大时紧急停止
    if (fabsf(offset_roll) > 40.0f) {
        left = 0.0f;
        right = 0.0f;
    }

    *left_out = fmaxf(-100.0f, fminf(100.0f, left));
    *right_out = fmaxf(-100.0f, fminf(100.0f, right));

                                   }
// ========== 参数设置接口 ==========
void attitude_set_roll_kp(float kp) { /* 滚转未使用，留空 */ }
void attitude_set_roll_kd(float kd) { /* 滚转未使用 */ }
void attitude_set_pitch_kp(float kp) { pid_pitch.kp = kp; ESP_LOGI(TAG, "Pitch KP=%.2f", kp); }
void attitude_set_pitch_kd(float kd) { pid_pitch.kd = kd; ESP_LOGI(TAG, "Pitch KD=%.2f", kd); }
void attitude_set_speed_pid(float flag,float kp, float ki, float kd) {
    if(flag==1){
    pid_speed.kp = kp;
    pid_speed.ki = ki;
    pid_speed.kd = kd;     
    }else if(flag==2){
    pid_pitch.kp = kp;
    pid_pitch.ki = ki;
    pid_pitch.kd = kd;   
    }else if(flag==3){
    pid_yaw_rate.kp = kp;
    pid_yaw_rate.ki = ki;
    pid_yaw_rate.kd = kd;   
    }

}
void attitude_set_yaw_rate_pid(float kp, float ki, float kd) {
    pid_yaw_rate.kp = kp;
    pid_yaw_rate.ki = ki;
    pid_yaw_rate.kd = kd;
    ESP_LOGI(TAG, "Yaw rate PID: KP=%.2f, KI=%.2f, KD=%.2f", kp, ki, kd);
}
void attitude_set_max_pitch(float max_pitch_deg) {
    if (max_pitch_deg > 0 && max_pitch_deg <= 90.0f) {
        max_pitch_cmd = max_pitch_deg;
        ESP_LOGI(TAG, "Max pitch set to %.1f deg", max_pitch_cmd);
    }
}
void attitude_clean_pid(void) {
    pid_pitch.integral = 0.0f;
    pid_pitch.prev_error = 0.0f;
    pid_speed.integral = 0.0f;
    pid_speed.prev_error = 0.0f;
    pid_yaw_rate.integral = 0.0f;
    pid_yaw_rate.prev_error = 0.0f;
}