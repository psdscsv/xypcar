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
#define GATTS_SERVICE_UUID 0xFFE0
#define GATTS_CHAR_CONTROL_UUID 0xFFE1
#define GATTS_NUM_HANDLE 8

// 数据包格式（25字节）
#define PKG_HEADER 0xA5
#define PKG_FOOTER 0x5A
#define PKG_TOTAL_LEN 25

// 包重组缓冲区
static uint8_t rx_buffer[PKG_TOTAL_LEN];
static size_t rx_index = 0;

static uint16_t gatts_service_handle = 0;
static uint16_t gatts_control_char_handle = 0;
static uint16_t gatts_conn_id = 0;
static bool is_connected = false;

// 函数声明
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// ========== 数据包解析（直接提取原始值） ==========
static bool parse_control_packet(const uint8_t *data, size_t len,
                                 int16_t *speed, int16_t *turn, int16_t *stop,
                                 float *turn_gain,
                                 float *speed_kp, float *speed_ki, float *speed_kd)
{
    if (len != PKG_TOTAL_LEN) return false;
    if (data[0] != PKG_HEADER) return false;
    if (data[PKG_TOTAL_LEN - 1] != PKG_FOOTER) return false;

    *speed = (int16_t)(data[1] | (data[2] << 8));
    *turn  = (int16_t)(data[3] | (data[4] << 8));
    *stop  = (int16_t)(data[5] | (data[6] << 8));

    memcpy(turn_gain, &data[7], 4);
    memcpy(speed_kp,  &data[11], 4);
    memcpy(speed_ki,  &data[15], 4);
    memcpy(speed_kd,  &data[19], 4);

    // 校验和：字节1 ~ 22
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

// ========== 写回调：解析后直接填充结构体，不做任何限制 ==========
static void handle_control_write(esp_ble_gatts_cb_param_t *param)
{
    if (!is_connected)
        return;
    uint8_t *value = param->write.value;
    size_t len = param->write.len;

    if (len == 0 || len > PKG_TOTAL_LEN)
    {
        ESP_LOGW(TAG, "Invalid fragment len=%d", len);
        return;
    }

    // 包头检测，重置缓冲区
    if (len > 0 && value[0] == PKG_HEADER)
    {
        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));
    }

    if (rx_index + len > PKG_TOTAL_LEN)
    {
        ESP_LOGW(TAG, "Buffer overflow, resetting");
        rx_index = 0;
        return;
    }

    memcpy(rx_buffer + rx_index, value, len);
    rx_index += len;

    if (rx_index == PKG_TOTAL_LEN)
    {
        if (rx_buffer[PKG_TOTAL_LEN - 1] != PKG_FOOTER)
        {
            ESP_LOGW(TAG, "Invalid footer");
            rx_index = 0;
            return;
        }

        int16_t sp, tr, st;
        float turn_g, kp, ki, kd;
    if (parse_control_packet(rx_buffer, PKG_TOTAL_LEN, &sp, &tr, &st,
                             &turn_g, &kp, &ki, &kd))
    {
        car_control_params_t params = {
            .stop = st,
            .target_speed = (float)sp,
            .target_turn = -(float)tr,
            .turn_gain = turn_g,
            .speed_pid_kp = kp,
            .speed_pid_ki = ki,
            .speed_pid_kd = kd,
        };
        car_control_update_params(&params);
    }
        else
        {
            ESP_LOGW(TAG, "Packet parse failed");
        }
        rx_index = 0;
    }
    else
    {
        ESP_LOGD(TAG, "Fragment received, %d/%d", rx_index, PKG_TOTAL_LEN);
    }
}

// ========== 蓝牙协议栈初始化（与之前相同） ==========
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

// ========== GATT 事件回调 ==========
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATT registered");
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {.inst_id = 0, .uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_SERVICE_UUID}}}};
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created, handle=%d", param->create.service_handle);
        gatts_service_handle = param->create.service_handle;
        esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_CHAR_CONTROL_UUID}};
        esp_ble_gatts_add_char(gatts_service_handle, &char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_CONTROL_UUID)
        {
            gatts_control_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Control characteristic added");
            esp_ble_gatts_start_service(gatts_service_handle);
        }
        break;

    case ESP_GATTS_START_EVT:
    {
        ESP_LOGI(TAG, "Service started, advertising...");
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE connected");
        is_connected = true;
        gatts_conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
    {
        ESP_LOGI(TAG, "BLE disconnected");
        is_connected = false;
        // 断开时发送零指令并紧急停止
        car_control_params_t zero = {0};
        car_control_update_params(&zero);
        // 重新广播
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == gatts_control_char_handle)
            handle_control_write(param);
        break;

    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT)
    {
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Advertising started");
        else
            ESP_LOGE(TAG, "Advertising start failed");
    }
}

void ble_control_init(void)
{
    ble_stack_init();
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_set_device_name("ESP32_Car");
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gatts_app_register(0);
    ESP_LOGI(TAG, "BLE control module initialized");
}
