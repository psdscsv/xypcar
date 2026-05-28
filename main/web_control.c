// car_manager.c
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
static const char *TAG = "WebCtrl";

// 单摇杆竖屏遥控界面 HTML

/**
 * @brief 非线性映射函数，将摇杆输入转换为实际输出百分比
 * @param input     -100 ~ 100，原始摇杆值
 * @param deadzone  死区阈值，绝对值小于此值输出0
 * @param min_out   最小输出百分比（当输入在死区边界时）
 * @param curve     曲线指数 (>1 使低段平缓)
 * @return int      映射后的输出（-100 ~ 100）
 */
static int map_joystick(int input, int deadzone, int min_out, float curve)
{
    int abs_in = abs(input);
    if (abs_in <= deadzone)
        return 0;
    int sign = (input > 0) ? 1 : -1;
    // 将 [deadzone, 100] 线性映射到 [0, 1]
    float t = (float)(abs_in - deadzone) / (100 - deadzone);
    // 应用幂曲线 t^curve，curve >1 使低段更平缓
    float mapped_t = powf(t, curve);
    // 映射到 [min_out, 100]
    int out_val = min_out + (int)(mapped_t * (100 - min_out));
    if (out_val > 100)
        out_val = 100;
    return sign * out_val;
}

/**
 * @brief 处理摇杆API请求
 *        GET /api/joystick?ly=xx&rx=xx
 */
static esp_err_t joystick_api_handler(httpd_req_t *req)
{
    char query[128] = {0};
    int ly = 0, rx = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    {
        char param_ly[8] = {0};
        char param_rx[8] = {0};
        if (httpd_query_key_value(query, "ly", param_ly, sizeof(param_ly)) == ESP_OK)
        {
            ly = atoi(param_ly);
        }
        if (httpd_query_key_value(query, "rx", param_rx, sizeof(param_rx)) == ESP_OK)
        {
            rx = atoi(param_rx);
        }
    }

    // 对速度指令（ly）应用非线性映射
    // 参数说明：死区15，最小输出20%，曲线指数1.5（可调）
    int ly_mapped = map_joystick(ly, 15, 20, 1.5);
    // 转向指令（rx）保持线性（也可根据需要映射）
    int rx_mapped = rx;

    // 限制范围
    if (ly_mapped > 100)
        ly_mapped = 100;
    if (ly_mapped < -100)
        ly_mapped = -100;
    if (rx_mapped > 100)
        rx_mapped = 100;
    if (rx_mapped < -100)
        rx_mapped = -100;

    // 混控算法：左轮 = 速度 + 转向，右轮 = 速度 - 转向
    int left_speed = ly_mapped + rx_mapped;
    int right_speed = ly_mapped - rx_mapped;

    // 限制最终输出
    if (left_speed > 100)
        left_speed = 100;
    if (left_speed < -100)
        left_speed = -100;
    if (right_speed > 100)
        right_speed = 100;
    if (right_speed < -100)
        right_speed = -100;

    float left_out, right_out;
    attitude_stabilize((float)ly_mapped, (float)rx_mapped, &left_out, &right_out);

    ESP_LOGI(TAG, "Joystick raw: ly=%d, rx=%d -> mapped ly=%d, rx=%d -> left=%d%%, right=%d%%",
             ly, rx, ly_mapped, rx_mapped, left_speed, right_speed);

    motor_set_speed((float)left_speed, (float)right_speed);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

/**
 * @brief 处理遥控页面请求
 */
static esp_err_t control_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CONTROL_HTML, strlen(CONTROL_HTML));
    return ESP_OK;
}

/**
 * @brief 初始化小车遥控管理模块
 *        注册 /control 和 /api/joystick 路由
 */
void web_control_init(void)
{
    httpd_handle_t server = wifi_get_http_server();
    if (server == NULL)
    {
        ESP_LOGE(TAG, "Web server not available! Make sure wifi_init() is called first.");
        return;
    }
    attitude_init();
    httpd_uri_t control_page = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = control_page_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &control_page);

    httpd_uri_t joystick_api = {
        .uri = "/api/joystick",
        .method = HTTP_GET,
        .handler = joystick_api_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &joystick_api);

    ESP_LOGI(TAG, "Car manager initialized. Control page: http://192.168.4.1/control");
}