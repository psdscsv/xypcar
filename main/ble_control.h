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
     * @brief 初始化BLE控制模块
     */
    void ble_control_init(void);

/**
     * @brief 发送数据到BLE客户端
     * @param data 数据指针
     * @param len 数据长度
     * @return ESP_OK表示成功，其他值表示失败
     */    
esp_err_t ble_control_send_data(const uint8_t *data, size_t len);
bool ble_control_is_connected(void);
#ifdef __cplusplus
}
#endif

#endif // BLE_CONTROL_H