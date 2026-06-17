// web_control.c
#include "web_control.h"
#include "motor_control.h"
#include "wifi_manager.h"
#include "web_page.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "attitude_control.h"
#include "car_control.h"          // 新增

static const char *TAG = "WebCtrl";

// 物理参数配置
#define MAX_LINEAR_SPEED_PPS   800.0f   // 最大线速度 (脉冲/秒)，根据实际电机最大转速调整
#define MAX_TURN_RATE_DPS      200.0f   // 最大转向角速度 (度/秒)

// 非线性映射函数（保持不变）
static int map_joystick(int input, int deadzone, int min_out, float curve) {
    int abs_in = abs(input);
    if (abs_in <= deadzone) return 0;
    int sign = (input > 0) ? 1 : -1;
    float t = (float)(abs_in - deadzone) / (100 - deadzone);
    float mapped_t = powf(t, curve);
    int out_val = min_out + (int)(mapped_t * (100 - min_out));
    if (out_val > 100) out_val = 100;
    return sign * out_val;
}

// 摇杆 API 处理函数
static esp_err_t joystick_api_handler(httpd_req_t *req) {
    char query[128] = {0};
    int ly = 0, rx = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param_ly[8] = {0};
        char param_rx[8] = {0};
        httpd_query_key_value(query, "ly", param_ly, sizeof(param_ly));
        httpd_query_key_value(query, "rx", param_rx, sizeof(param_rx));
        ly = atoi(param_ly);
        rx = -atoi(param_rx);
    }

    // 非线性映射（死区15，最小输出20%，曲线指数1.5）
    int ly_mapped = map_joystick(ly, 15, 20, 1.5);
    int rx_mapped = map_joystick(rx, 10, 10, 1.2);   // 转向也可适当映射

    // 转换为目标值
    float target_speed = (float)ly_mapped / 100.0f * MAX_LINEAR_SPEED_PPS;
    float target_turn  = (float)rx_mapped / 100.0f * MAX_TURN_RATE_DPS;

    // 构建控制参数
    car_control_params_t params = {
        .stop          = 0,    // 只要有输入就不停止
        .target_speed  = target_speed,
        .target_turn   = target_turn,
        .turn_gain     = 0.0f,
        .speed_pid_kp  = 0.0f,
        .speed_pid_ki  = 0.0f,
        .speed_pid_kd  = 0.0f,
    };

    // 更新到小车控制模块
    car_control_update_params(&params);

    ESP_LOGI(TAG, "Web: ly=%d -> speed=%.1f pps, rx=%d -> turn=%.1f dps, stop=%d",
             ly_mapped, target_speed, rx_mapped, target_turn, params.stop);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// 遥控页面处理（不变）
static esp_err_t control_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CONTROL_HTML, strlen(CONTROL_HTML));
    return ESP_OK;
}

// Web 控制初始化
void web_control_init(void) {
    httpd_handle_t server = wifi_get_http_server();
    if (server == NULL) {
        ESP_LOGE(TAG, "Web server not available! Make sure wifi_init() is called first.");
        return;
    }

    httpd_uri_t control_page = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = control_page_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &control_page);

    httpd_uri_t joystick_api = {
        .uri       = "/api/joystick",
        .method    = HTTP_GET,
        .handler   = joystick_api_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &joystick_api);

    ESP_LOGI(TAG, "Web control initialized. Access http://192.168.4.1/control");
}