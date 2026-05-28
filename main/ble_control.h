// ble_control.h
#ifndef BLE_CONTROL_H
#define BLE_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化BLE控制模块
     */
    void ble_control_init(void);

    /**
     * @brief 获取当前目标速度与转向（由BLE数据更新）
     * @param speed 输出目标速度 [-100, 100]
     * @param turn  输出目标转向 [-100, 100]
     */
    void ble_get_target(float *speed, float *turn);

#ifdef __cplusplus
}
#endif

#endif // BLE_CONTROL_H