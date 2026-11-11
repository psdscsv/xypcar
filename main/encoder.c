// encoder.c
#include "encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_control.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "Encoder";

// 引脚定义
#define ENC_LEFT_A_GPIO  8
#define ENC_LEFT_B_GPIO  9
#define ENC_RIGHT_A_GPIO 10
#define ENC_RIGHT_B_GPIO 11

#define TWO_PI 6.283185307f

// 全局脉冲计数（中断中修改，使用临界区保护）
static volatile int s_left_pulse = 0;
static volatile int s_right_pulse = 0;

// 上次读取的脉冲值和时间（用于速度计算）
static int s_last_left_pulse = 0;
static int s_last_right_pulse = 0;
static uint32_t s_last_time_ms = 0;

// 临界区保护变量
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// 正交解码状态
static volatile uint8_t s_left_last_state = 0;
static volatile uint8_t s_right_last_state = 0;

// 正交解码增量表
static const int8_t s_quadrature_table[4][4] = {
    {0, 1, -1, 0},
    {-1, 0, 0, 1},
    {1, 0, 0, -1},
    {0, -1, 1, 0}
};

// 通用中断服务
static void IRAM_ATTR encoder_isr(gpio_num_t gpio_a, gpio_num_t gpio_b,
                                  volatile int *pulse_counter, volatile uint8_t *last_state)
{
    uint8_t a = gpio_get_level(gpio_a);
    uint8_t b = gpio_get_level(gpio_b);
    uint8_t curr_state = (a << 1) | b;

    int8_t inc = s_quadrature_table[*last_state][curr_state];
    if (inc != 0)
    {
        portENTER_CRITICAL_ISR(&s_mux);
        *pulse_counter += inc;
        portEXIT_CRITICAL_ISR(&s_mux);
    }
    *last_state = curr_state;
}

static void IRAM_ATTR left_isr_handler(void *arg)
{
    encoder_isr(ENC_LEFT_A_GPIO, ENC_LEFT_B_GPIO, &s_left_pulse, &s_left_last_state);
}

static void IRAM_ATTR right_isr_handler(void *arg)
{
    encoder_isr(ENC_RIGHT_A_GPIO, ENC_RIGHT_B_GPIO, &s_right_pulse, &s_right_last_state);
}

bool encoder_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENC_LEFT_A_GPIO) | (1ULL << ENC_LEFT_B_GPIO) |
                        (1ULL << ENC_RIGHT_A_GPIO) | (1ULL << ENC_RIGHT_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_intr_type(ENC_LEFT_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENC_LEFT_B_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENC_RIGHT_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENC_RIGHT_B_GPIO, GPIO_INTR_ANYEDGE);

    static bool isr_installed = false;
    if (!isr_installed)
    {
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        isr_installed = true;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_LEFT_A_GPIO, left_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_LEFT_B_GPIO, left_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_RIGHT_A_GPIO, right_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_RIGHT_B_GPIO, right_isr_handler, NULL));

    s_left_last_state = (gpio_get_level(ENC_LEFT_A_GPIO) << 1) | gpio_get_level(ENC_LEFT_B_GPIO);
    s_right_last_state = (gpio_get_level(ENC_RIGHT_A_GPIO) << 1) | gpio_get_level(ENC_RIGHT_B_GPIO);

    s_last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Encoder initialized (4x quadrature), raw PPR=%d, resolution=%d pulses/rev",
             PPR_RAW, PPR_RAW * 4);
    return true;
}

void encoder_get_pulse(int *left, int *right) {
    int left_val, right_val;
    portENTER_CRITICAL(&s_mux);
    left_val = s_left_pulse;
    right_val = s_right_pulse;
    portEXIT_CRITICAL(&s_mux);
    if (left) *left = left_val;
    if (right) *right = -right_val;   // 强制右轮符号与左轮一致
}

/**
 * 累计行驶距离：把脉冲计数换算成米。
 * 每个正交脉冲对应的轮子位移 = (2πR) / (PPR * 4)
 */
void encoder_get_distance(float *left_dist_m, float *right_dist_m)
{
    if (!left_dist_m || !right_dist_m) return;

    int left_pulse, right_pulse;
    encoder_get_pulse(&left_pulse, &right_pulse);

    const float dist_per_pulse = (TWO_PI * WHEEL_RADIUS) / (PPR_RAW * 4.0f);

    *left_dist_m  = (float)left_pulse  * dist_per_pulse;
    *right_dist_m = (float)right_pulse * dist_per_pulse;
}

void encoder_get_speed(float *left_speed_ms, float *right_speed_ms)
{
    if (!left_speed_ms || !right_speed_ms)
        return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - s_last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f)
    {
        *left_speed_ms = 0;
        *right_speed_ms = 0;
        s_last_time_ms = now_ms;
        return;
    }

    int left_pulse, right_pulse;
    encoder_get_pulse(&left_pulse, &right_pulse);

    int left_delta = left_pulse - s_last_left_pulse;
    int right_delta = right_pulse - s_last_right_pulse;

    float pulses_per_rev = PPR_RAW * 4.0f;
    float wheel_circumference = TWO_PI * WHEEL_RADIUS;
    *left_speed_ms = (left_delta / pulses_per_rev) * wheel_circumference / dt;
    *right_speed_ms = (right_delta / pulses_per_rev) * wheel_circumference / dt;

    s_last_left_pulse = left_pulse;
    s_last_right_pulse = right_pulse;
    s_last_time_ms = now_ms;

    ESP_LOGD(TAG, "left_speed_ms=%f, right_speed_ms=%f",
             *left_speed_ms, *right_speed_ms);

    /* ===== 上报累计距离（50Hz/2 = 25Hz）=====
     * 报文格式: "D:<left_m>,<right_m>"
     * 保留 4 位小数（0.1mm 精度），供手机端航位推算。
     */
    static int counter = 0;
    if (++counter % 2 == 0) {
        float ldist, rdist;
        encoder_get_distance(&ldist, &rdist);
        char dist_data[48];
        snprintf(dist_data, sizeof(dist_data), "D:%.4f,%.4f", ldist, rdist);
        ble_control_send_data((uint8_t*)dist_data, strlen(dist_data));
    }
}