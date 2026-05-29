#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        // 实时指令
        int stop;
        float target_speed;
        float target_turn;

        // 原有增益
        float kp_roll;
        float kp_pitch;
        float speed_pitch_gain;
        float turn_gain;

        // 新增增益
        float roll_turn_gain;    // 转向→期望滚转增益
        float turn_speed_factor; // 速度‑转向调度系数
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