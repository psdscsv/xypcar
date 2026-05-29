// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化编码器（PCNT）
     *        左电机：GPIO8 (A), GPIO9 (B)
     *        右电机：GPIO10 (A), GPIO20 (B)
     * @param ppr 编码器每转脉冲数（PPR，如11、13、7等）
     * @return true 成功, false 失败
     */
    bool encoder_init(void);

    /**
     * @brief 获取左右电机累计脉冲计数（有符号，正转增加，反转减少）
     * @param left  左电机脉冲数
     * @param right 右电机脉冲数
     */
    void encoder_get_pulse(int *left, int *right);

    /**
     * @brief 清空左右电机脉冲计数（归零）
     */
    void encoder_reset_pulse(void);

    /**
     * @brief 获取左右电机的瞬时转速（基于最近两次读取的时间差）
     *        需要周期性调用（建议20ms调用一次），内部自动计算速度
     * @param left_speed_pps  左电机转速，单位：脉冲/秒
     * @param right_speed_pps 右电机转速，单位：脉冲/秒
     * @note  第一次调用时速度为零，以后每次调用根据两次间隔和脉冲差计算
     */
    void encoder_get_speed(float *left_speed_pps, float *right_speed_pps);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H