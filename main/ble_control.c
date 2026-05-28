// ble_control.c
#include "ble_control.h"
#include "motor_control.h"
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

// 新数据包格式（25字节）:
// [0xA5] [speed_low] [speed_high] [turn_low] [turn_high] [stop_low] [stop_high]
// [kp_roll (float 4B)] [turn_gain (float 4B)] [kp_pitch (float 4B)] [speed_pitch_gain (float 4B)]
// [checksum] [0x5A]
#define PKG_HEADER 0xA5
#define PKG_FOOTER 0x5A
#define PKG_TOTAL_LEN 25

#define CONTROL_TASK_PERIOD_MS 20

// 包重组缓冲区
static uint8_t rx_buffer[PKG_TOTAL_LEN];
static size_t rx_index = 0;

// 全局可调参数
static float g_turn_gain = 1.0f; // 转向增益，默认1.0
static float target_speed = 0.0f;
static float target_turn = 0.0f;
static int16_t stop = 0; // 0: 正常运行, 非0: 紧急停止
static SemaphoreHandle_t target_mutex = NULL;

static uint16_t gatts_service_handle = 0;
static uint16_t gatts_control_char_handle = 0;
static uint16_t gatts_conn_id = 0;
static bool is_connected = false;

// 函数声明
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void control_task(void *pvParameters);

// ========== 数据包解析（新格式 25 字节） ==========
static bool parse_control_packet(const uint8_t *data, size_t len,
                                 int16_t *speed, int16_t *turn, int16_t *stop,
                                 float *kp_roll, float *turn_gain,
                                 float *kp_pitch, float *speed_pitch_gain)
{
    if (len != PKG_TOTAL_LEN)
        return false;
    if (data[0] != PKG_HEADER)
        return false;
    if (data[PKG_TOTAL_LEN - 1] != PKG_FOOTER)
        return false;

    // 解析整数部分
    *speed = (int16_t)(data[1] | (data[2] << 8));
    *turn = (int16_t)(data[3] | (data[4] << 8));
    *stop = (int16_t)(data[5] | (data[6] << 8));

    // 解析 float 部分（小端序）
    memcpy(kp_roll, &data[7], 4);
    memcpy(turn_gain, &data[11], 4);
    memcpy(kp_pitch, &data[15], 4);
    memcpy(speed_pitch_gain, &data[19], 4);

    // 校验和：对 data[1] 到 data[23] 求和（共 23 字节）
    uint8_t calc_checksum = 0;
    for (int i = 1; i < PKG_TOTAL_LEN - 2; i++)
    {
        calc_checksum += data[i];
    }
    uint8_t recv_checksum = data[PKG_TOTAL_LEN - 2];
    if (calc_checksum != recv_checksum)
    {
        ESP_LOGW(TAG, "Checksum mismatch: calc=0x%02X, recv=0x%02X", calc_checksum, recv_checksum);
        return false;
    }
    return true;
}

// ========== 写回调处理（支持分片重组） ==========
static void handle_control_write(esp_ble_gatts_cb_param_t *param)
{
    if (!is_connected)
        return;
    uint8_t *value = param->write.value;
    size_t len = param->write.len;

    // 安全检查
    if (len == 0 || len > PKG_TOTAL_LEN)
    {
        ESP_LOGW(TAG, "Invalid fragment len=%d", len);
        return;
    }

    // 如果收到包头，说明是新包的开始，重置缓冲区
    if (len > 0 && value[0] == PKG_HEADER)
    {
        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));
    }

    // 防止缓冲区溢出
    if (rx_index + len > PKG_TOTAL_LEN)
    {
        ESP_LOGW(TAG, "Buffer overflow, resetting");
        rx_index = 0;
        return;
    }

    // 复制数据到缓冲区
    memcpy(rx_buffer + rx_index, value, len);
    rx_index += len;

    // 检查是否收齐完整包
    if (rx_index == PKG_TOTAL_LEN)
    {
        // 验证包尾是否为 0x5A
        if (rx_buffer[PKG_TOTAL_LEN - 1] != PKG_FOOTER)
        {
            ESP_LOGW(TAG, "Invalid footer, discarding packet");
            rx_index = 0;
            return;
        }

        int16_t sp, tr, st;
        float kp_r, turn_g, kp_p, sp_gain;
        if (parse_control_packet(rx_buffer, PKG_TOTAL_LEN, &sp, &tr, &st,
                                 &kp_r, &turn_g, &kp_p, &sp_gain))
        {
            // 更新参数
            if (kp_r > 0.0f && kp_r < 100.0f)
            {
                attitude_set_roll_kp(kp_r);
            }
            if (kp_p > 0.0f && kp_p < 100.0f)
            {
                attitude_set_pitch_kp(kp_p);
            }
            if (sp_gain >= 0.0f && sp_gain <= 1.0f)
            {
                attitude_set_speed_to_pitch_gain(sp_gain);
            }
            if (turn_g >= 0.0f && turn_g <= 2.0f)
            {
                g_turn_gain = turn_g;
                ESP_LOGI(TAG, "Turn gain updated: %.2f", g_turn_gain);
            }

            stop = st;
            if (st != 0)
            {
                if (target_mutex)
                {
                    xSemaphoreTake(target_mutex, portMAX_DELAY);
                    target_speed = 0.0f;
                    target_turn = 0.0f;
                    xSemaphoreGive(target_mutex);
                }
                motor_set_speed(0, 0);
                ESP_LOGI(TAG, "Emergency stop");
                rx_index = 0;
                return;
            }

            // 限幅
            if (sp < -100)
                sp = -100;
            if (sp > 100)
                sp = 100;
            if (tr < -100)
                tr = -100;
            if (tr > 100)
                tr = 100;

            // 应用转向增益
            float turn_adjusted = (float)tr * g_turn_gain;
            if (turn_adjusted > 100)
                turn_adjusted = 100;
            if (turn_adjusted < -100)
                turn_adjusted = -100;

            if (target_mutex)
            {
                xSemaphoreTake(target_mutex, portMAX_DELAY);
                target_speed = (float)sp;
                target_turn = turn_adjusted;
                xSemaphoreGive(target_mutex);
            }
            ESP_LOGI(TAG, "Speed=%d, Turn=%.0f, Stop=%d, KpR=%.2f, KpP=%.2f, SpGain=%.2f, TG=%.2f",
                     sp, turn_adjusted, st, kp_r, kp_p, sp_gain, g_turn_gain);
        }
        else
        {
            ESP_LOGW(TAG, "Packet parse failed");
        }
        // 重置缓冲区，准备接收下一包
        rx_index = 0;
    }
    else
    {
        // 尚未收齐，等待下一个分片
        ESP_LOGD(TAG, "Received fragment, total so far: %d/%d", rx_index, PKG_TOTAL_LEN);
    }
}

// ========== 控制任务（20ms 周期） ==========
static void control_task(void *pvParameters)
{
    float spd, trn;
    float left_out, right_out;
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        if (target_mutex)
        {
            xSemaphoreTake(target_mutex, portMAX_DELAY);
            spd = target_speed;
            trn = target_turn;
            xSemaphoreGive(target_mutex);
        }
        else
        {
            spd = 0;
            trn = 0;
        }

        // 调用姿态稳定控制（闭环）
        attitude_stabilize(spd, trn, &left_out, &right_out);

        // 紧急停止检查
        if (stop != 0)
        {
            left_out = 0;
            right_out = 0;
        }

        motor_set_speed(left_out, right_out);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
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

// ========== GATT 事件回调 ==========
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATT registered");
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_SERVICE_UUID}}}};
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created, handle=%d", param->create.service_handle);
        gatts_service_handle = param->create.service_handle;
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GATTS_CHAR_CONTROL_UUID}};
        esp_ble_gatts_add_char(gatts_service_handle, &char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_CONTROL_UUID)
        {
            gatts_control_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Control characteristic added, handle=%d", gatts_control_char_handle);
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
        motor_set_speed(0, 0);
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
        {
            handle_control_write(param);
        }
        break;

    default:
        break;
    }
}

// ========== GAP 事件回调 ==========
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Advertising started");
        else
            ESP_LOGE(TAG, "Advertising start failed");
        break;
    default:
        break;
    }
}

// ========== 公共初始化 ==========
void ble_control_init(void)
{
    target_mutex = xSemaphoreCreateMutex();
    if (!target_mutex)
    {
        ESP_LOGE(TAG, "Mutex creation failed");
        return;
    }

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

    xTaskCreate(control_task, "ctrl_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "BLE control module initialized");
}

// 获取当前目标（保留接口）
void ble_get_target(float *speed, float *turn)
{
    if (target_mutex)
    {
        xSemaphoreTake(target_mutex, portMAX_DELAY);
        *speed = target_speed;
        *turn = target_turn;
        xSemaphoreGive(target_mutex);
    }
    else
    {
        *speed = 0;
        *turn = 0;
    }
}