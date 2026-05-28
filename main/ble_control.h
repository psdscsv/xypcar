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
     *        - 初始化蓝牙协议栈
     *        - 创建GATT服务（控制写特征、姿态通知特征）
     *        - 创建控制任务（20ms周期调用姿态稳定控制）
     *        - 创建姿态通知任务（可选，每100ms发送一次姿态角）
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