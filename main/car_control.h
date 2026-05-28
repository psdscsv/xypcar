#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 控制参数结构体（包含所有可调参数和指令）
    typedef struct
    {
        // 实时控制指令
        int stop;           // 0: 正常，1: 紧急停止
        float target_speed; // -100 ~ 100，原始值（未经任何调整）
        float target_turn;  // -100 ~ 100，原始值

        // 可调参数
        float kp_roll;          // 滚转比例系数
        float kp_pitch;         // 俯仰比例系数
        float speed_pitch_gain; // 速度->俯仰角增益
        float turn_gain;        // 转向增益（用于调整转向灵敏度）

    } car_control_params_t;

    /**
     * @brief 初始化小车控制模块
     */
    void car_control_init(void);

    /**
     * @brief 外部更新控制参数（由 BLE 或其他模块调用）
     * @param params 指向新的控制参数结构体（包含指令和增益）
     */
    void car_control_update_params(const car_control_params_t *params);

    /**
     * @brief 获取当前控制参数（带互斥保护）
     */
    void car_control_get_params(car_control_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H