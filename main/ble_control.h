// ble_control.h
#ifndef BLE_CONTROL_H
#define BLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化 BLE 控制模块
     *        - 注册 GAP/GATT 回调
     *        - 启动广播
     *        - 创建看门狗任务（处理假连接、广播丢失）
     */
    void ble_control_init(void);

    /**
     * @brief 向 BLE 客户端发送数据（Notify）
     * @param data 数据指针
     * @param len  数据长度
     * @return ESP_OK 表示成功；未连接或未订阅时返回 ESP_FAIL
     */
    esp_err_t ble_control_send_data(const uint8_t *data, size_t len);

    /**
     * @brief 当前是否存在已连接（且已订阅通知）的客户端
     */
    bool ble_control_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_CONTROL_H