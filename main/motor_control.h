// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>

/**
 * @brief 初始化电机驱动
 *        - 配置 PWM 通道
 *        - 设置使能引脚（GPIO1）为高电平
 *        - 停止电机
 */
void motor_init(void);

/**
 * @brief 设置左右电机速度（百分比）
 * @param left_percent   -100.0 ~ 100.0，正数前进，负数后退
 * @param right_percent  -100.0 ~ 100.0
 */
void motor_set_speed(float left_percent, float right_percent);

/**
 * @brief 紧急停止电机（速度归零）
 */
void motor_emergency_stop(void);

#endif // MOTOR_CONTROL_H