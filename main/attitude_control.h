// attitude_control.h
#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化姿态控制（MPU6050 已初始化）
     */
    void attitude_init(void);

    /**
     * @brief 获取当前姿态角（单位：度）
     * @param roll  滚转角（左右倾斜），正向右倾
     * @param pitch 俯仰角（前后倾斜），正向抬头
     */
    void attitude_get_angles(float *roll, float *pitch);

    /**
     * @brief 获取当前偏航角速度（单位：度/秒）
     * @param yaw_rate 输出偏航角速度
     */
    void attitude_get_yaw_rate(float *yaw_rate);

    /**
     * @brief 姿态与速度/角速度稳定控制（级联控制，推荐使用）
     * @param target_linear_speed   目标线速度 (m/s)
     * @param target_angular_rate   目标转向角速度 (度/秒，正右转)
     * @param current_left_speed    当前左轮线速度 (m/s)
     * @param current_right_speed   当前右轮线速度 (m/s)
     * @param left_out              输出左电机百分比 (-100 ~ 100)
     * @param right_out             输出右电机百分比
     */
    void attitude_stabilize_with_speed(float target_linear_speed, float target_angular_rate,
                                       float current_left_speed, float current_right_speed,
                                       float *left_out, float *right_out);

    /**
     * @brief 动态参数设置（用于 BLE/Web 调参）
     */
    void attitude_set_roll_kp(float kp);
    void attitude_set_pitch_kp(float kp);
    void attitude_set_roll_kd(float kd);
    void attitude_set_pitch_kd(float kd);
    void attitude_set_speed_pid(float kp, float ki, float kd);    // 线速度外环
    void attitude_set_yaw_rate_pid(float kp, float ki, float kd); // 角速度外环
    void attitude_set_speed_to_pitch_gain(float gain);            // 已弃用，保留兼容
    void attitude_clean_pid(void);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_CONTROL_H