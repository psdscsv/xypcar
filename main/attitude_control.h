// attitude_control.h
#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化姿态控制（MPU6050 已初始化，此函数校准并启动）
     */
    void attitude_init(void);

    /**
     * @brief 获取当前姿态角（单位：度）
     * @param roll  滚转角（左右倾斜），正向右倾
     * @param pitch 俯仰角（前后倾斜），正向抬头
     */
    void attitude_get_angles(float *roll, float *pitch);

    /**
     * @brief 姿态稳定控制：根据目标速度和转向，以及当前姿态，输出修正后的电机速度
     * @param target_speed  目标前进速度（-100 ~ 100）
     * @param target_turn   目标转向（-100 ~ 100，负左正右）
     * @param left_out      输出左电机速度（-100 ~ 100）
     * @param right_out     输出右电机速度（-100 ~ 100）
     */
    void attitude_stabilize(float target_speed, float target_turn,
                            float *left_out, float *right_out);
    void attitude_set_roll_kp(float kp);
#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_CONTROL_H