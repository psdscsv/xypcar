// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

// 编码器原始PPR（每转脉冲数，未经倍频）
#define PPR_RAW 69
// 轮子半径（米）
#define WHEEL_RADIUS 0.1f
// 最大线速度（米/秒）
#define MAX_SPEED 10.0f  

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化编码器
     *        左电机：GPIO8 (A), GPIO9 (B)
     *        右电机：GPIO10 (A), GPIO11 (B)
     * @return true 成功, false 失败
     */
    bool encoder_init(void);

    /**
     * @brief 获取左右电机累计脉冲计数（有符号，正转增加，反转减少）
     */
    void encoder_get_pulse(int *left, int *right);

    /**
     * @brief 获取左右电机的瞬时转速（m/s）
     *        需要周期性调用（建议20ms调用一次），内部自动计算速度
     *        同时会按 25Hz 向 BLE 上报累计距离（"D:xxx,xxx"）
     */
    void encoder_get_speed(float *left_speed_ms, float *right_speed_ms);

    /**
     * @brief 获取左右轮累计行驶距离（有符号，单位：米）
     *        前进为正，后退为负。供手机端做航位推算。
     */
    void encoder_get_distance(float *left_dist_m, float *right_dist_m);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H