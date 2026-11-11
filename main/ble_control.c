// ble_control.c
#include "ble_control.h"
#include "car_control.h"
#include "attitude_control.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "BLECtrl";

// ========== GATT 定义 ==========
#define GATTS_SERVICE_UUID      0xFFE0
#define GATTS_CHAR_CONTROL_UUID 0xFFE1
#define GATTS_NUM_HANDLE        8

// 数据包格式（25 字节）
#define PKG_HEADER    0xA5
#define PKG_FOOTER    0x5A
#define PKG_TOTAL_LEN 25

// 看门狗配置
#define BLE_WD_PERIOD_MS        2000

// ========== 内部状态 ==========
static uint8_t  rx_buffer[PKG_TOTAL_LEN];
static size_t   rx_index = 0;

static uint16_t gatts_service_handle       = 0;
static uint16_t gatts_control_char_handle  = 0;
static uint16_t gatts_control_desc_handle  = 0;
static uint16_t gatts_conn_id              = 0;
static esp_gatt_if_t s_gatts_if            = 0;

static volatile bool     is_connected     = false;
static volatile bool     s_notify_enabled = false;
static volatile bool     s_adv_running    = false;
static volatile uint32_t s_last_rx_ms     = 0;

// 函数声明
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);
static void ble_start_advertising(void);

// ========== 数据包解析 ==========
static bool parse_control_packet(const uint8_t *data, size_t len,
                                 int16_t *speed, int16_t *turn, int16_t *stop,
                                 float *pid_flag,
                                 float *speed_kp, float *speed_ki, float *speed_kd)
{
    if (len != PKG_TOTAL_LEN) return false;
    if (data[0] != PKG_HEADER) return false;
    if (data[PKG_TOTAL_LEN - 1] != PKG_FOOTER) return false;

    *speed = (int16_t)(data[1] | (data[2] << 8));
    *turn  = (int16_t)(data[3] | (data[4] << 8));
    *stop  = (int16_t)(data[5] | (data[6] << 8));

    memcpy(pid_flag, &data[7],  4);
    memcpy(speed_kp, &data[11], 4);
    memcpy(speed_ki, &data[15], 4);
    memcpy(speed_kd, &data[19], 4);

    uint8_t calc_checksum = 0;
    for (int i = 1; i <= 22; i++) {
        calc_checksum += data[i];
    }
    if (calc_checksum != data[23]) {
        ESP_LOGW(TAG, "Checksum mismatch");
        return false;
    }
    return true;
}

// ========== 写回调 ==========
static void handle_control_write(esp_ble_gatts_cb_param_t *param)
{
    if (!is_connected) return;

    uint8_t *value = param->write.value;
    size_t   len   = param->write.len;

    if (len == 0 || len > PKG_TOTAL_LEN) {
        ESP_LOGW(TAG, "Invalid fragment len=%d", (int)len);
        return;
    }

    if (value[0] == PKG_HEADER) {
        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));
    }

    if (rx_index + len > PKG_TOTAL_LEN) {
        ESP_LOGW(TAG, "Buffer overflow, resetting");
        rx_index = 0;
        return;
    }

    memcpy(rx_buffer + rx_index, value, len);
    rx_index += len;

    if (rx_index == PKG_TOTAL_LEN) {
        if (rx_buffer[PKG_TOTAL_LEN - 1] != PKG_FOOTER) {
            ESP_LOGW(TAG, "Invalid footer");
            rx_index = 0;
            return;
        }

        int16_t sp, tr, st;
        float pid_s, kp, ki, kd;
        if (parse_control_packet(rx_buffer, PKG_TOTAL_LEN,
                                 &sp, &tr, &st,
                                 &pid_s, &kp, &ki, &kd)) {
            car_control_params_t params = {
                .stop         = st,
                .target_speed = (float)sp / 100.0f,
                .target_turn  = -(float)tr,
                .pid_flag     = pid_s,
                .pid_kp       = kp,
                .pid_ki       = ki,
                .pid_kd       = kd,
            };
            car_control_update_params(&params);
        } else {
            ESP_LOGW(TAG, "Packet parse failed");
        }
        rx_index = 0;
    }
}

// ========== 蓝牙协议栈初始化 ==========
static void ble_stack_init(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_LOGI(TAG, "Bluetooth stack initialized");
}

// ========== 启动广播 ==========
static void ble_start_advertising(void)
{
    esp_ble_adv_params_t adv_params = {
        .adv_int_min       = 0x20,
        .adv_int_max       = 0x40,
        .adv_type          = ADV_TYPE_IND,
        .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
        .channel_map       = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret == ESP_OK) {
        s_adv_running = true;
        ESP_LOGI(TAG, "Advertising started");
    } else if (ret == ESP_ERR_INVALID_STATE) {
        s_adv_running = true;
        ESP_LOGI(TAG, "Advertising already running");
    } else {
        ESP_LOGE(TAG, "start_advertising failed: %s", esp_err_to_name(ret));
        s_adv_running = false;
    }
}

// ========== 看门狗任务 ==========
static void ble_watchdog_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BLE_WD_PERIOD_MS));

        // 只处理广播丢失；连接断开交给 BLE 协议栈
        if (!is_connected && !s_adv_running) {
            ESP_LOGW(TAG, "广播丢失，重新开启");
            ble_start_advertising();
        }
    }
}

// ========== GATT 事件回调 ==========
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        s_gatts_if = gatts_if;
        ESP_LOGI(TAG, "GATT registered, if=%d", s_gatts_if);
        {
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {.len = ESP_UUID_LEN_16,
                             .uuid = {.uuid16 = GATTS_SERVICE_UUID}}
                }
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        }
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created, handle=%d", param->create.service_handle);
        gatts_service_handle = param->create.service_handle;
        {
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_CONTROL_UUID}
            };
            esp_ble_gatts_add_char(
                gatts_service_handle, &char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_WRITE |
                ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
                ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                ESP_GATT_CHAR_PROP_BIT_INDICATE,
                NULL, NULL);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_CONTROL_UUID) {
            gatts_control_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Control characteristic added, handle=%d",
                     gatts_control_char_handle);
            esp_bt_uuid_t desc_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
            };
            esp_ble_gatts_add_char_descr(
                gatts_service_handle, &desc_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL, NULL);
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.descr_uuid.uuid.uuid16 ==
            ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            gatts_control_desc_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCC descriptor added, handle=%d",
                     gatts_control_desc_handle);
            esp_ble_gatts_start_service(gatts_service_handle);
        }
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "Service started, advertising...");
        ble_start_advertising();
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE connected, conn_id=%d", param->connect.conn_id);
        is_connected     = true;
        s_notify_enabled = false;
        s_adv_running    = false;
        gatts_conn_id    = param->connect.conn_id;
        s_last_rx_ms     = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "BLE disconnected, reason=0x%x", param->disconnect.reason);
        is_connected     = false;
        s_notify_enabled = false;
        rx_index         = 0;
        s_adv_running    = false;

        {
            car_control_params_t stop_cmd = {
                .stop         = 1,
                .target_speed = 0,
                .target_turn  = 0,
                .pid_flag     = 0,
                .pid_kp       = 0,
                .pid_ki       = 0,
                .pid_kd       = 0,
            };
            car_control_update_params(&stop_cmd);
        }

        ble_start_advertising();
        break;

    case ESP_GATTS_WRITE_EVT:
        s_last_rx_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        
    ESP_LOGI(TAG, "WRITE_EVT handle=%d need_rsp=%d len=%d",
             param->write.handle, param->write.need_rsp, param->write.len);

    if (param->write.need_rsp) {
        esp_err_t ret = esp_ble_gatts_send_response(
            gatts_if,
            param->write.conn_id,
            param->write.trans_id,
            ESP_GATT_OK,
            NULL);
        ESP_LOGI(TAG, "send_response ret=%s", esp_err_to_name(ret));
    }
        if (param->write.handle == gatts_control_desc_handle) {
            // 手机写 CCC，判定订阅状态
            if (param->write.len >= 2) {
                uint16_t ccc = (uint16_t)(param->write.value[0] |
                                          (param->write.value[1] << 8));
                s_notify_enabled = (ccc & 0x0001) != 0;
                ESP_LOGI(TAG, "CCC updated: 0x%04x, notify=%s",
                         ccc, s_notify_enabled ? "ON" : "OFF");
            }
        } else if (param->write.handle == gatts_control_char_handle) {
            handle_control_write(param);
        }
        break;

    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            s_adv_running = true;
            ESP_LOGI(TAG, "Advertising started (GAP cb)");
        } else {
            s_adv_running = false;
            ESP_LOGE(TAG, "Advertising start failed, status=%d",
                     param->adv_start_cmpl.status);
        }
    } else if (event == ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT) {
        s_adv_running = false;
        ESP_LOGI(TAG, "Advertising stopped");
    }
}

// ========== 初始化 ==========
void ble_control_init(void)
{
    ble_stack_init();
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_set_device_name("ESP32_Car");

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp    = false,
        .include_name    = true,
        .include_txpower = false,
        .flag            = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    esp_ble_gap_config_adv_data(&adv_data);

    esp_ble_gatts_app_register(0);

    xTaskCreate(ble_watchdog_task, "ble_wd", 3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "BLE control module initialized");
}

// ========== 发送数据（Notify） ==========
esp_err_t ble_control_send_data(const uint8_t *data, size_t len)
{
    if (!is_connected) {
        ESP_LOGW(TAG, "Not connected");
        return ESP_FAIL;
    }
    if (!s_notify_enabled) {
        ESP_LOGW(TAG, "Notify not enabled by peer");
        return ESP_FAIL;
    }
    if (gatts_control_char_handle == 0) {
        ESP_LOGW(TAG, "Characteristic handle invalid");
        return ESP_FAIL;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_ble_gatts_send_indicate(
        s_gatts_if,
        gatts_conn_id,
        gatts_control_char_handle,
        (uint16_t)len,
        (uint8_t *)data,
        false);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send indicate failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

bool ble_control_is_connected(void)
{
    return is_connected && s_notify_enabled;
}