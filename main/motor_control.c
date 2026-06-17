// motor_control.c
#include "motor_control.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR";

// ========== 引脚定义 ==========
#define M1_IN1_GPIO 4
#define M1_IN2_GPIO 5
#define M2_IN3_GPIO 6
#define M2_IN4_GPIO 7
#define DRV8833_NSLEEP 3

// ========== LEDC 配置 ==========
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ 5000
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // 0-255

#define CH_M1_FWD LEDC_CHANNEL_0
#define CH_M1_REV LEDC_CHANNEL_1
#define CH_M2_FWD LEDC_CHANNEL_2
#define CH_M2_REV LEDC_CHANNEL_3

/**
 * @brief 内部函数：将百分比转换为占空比值（0-255）
 */
static int percent_to_duty(float percent)
{
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;
    return (int)(percent * 255.0f / 100.0f + 0.5f);
}

void motor_init(void)
{
    // 1. 配置并使能 DRV8833（NSLEEP = 高电平）
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << DRV8833_NSLEEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&en_conf);
    gpio_set_level(DRV8833_NSLEEP, 1); // 使能芯片
    ESP_LOGI(TAG, "DRV8833 enabled (NSLEEP=GPIO%d)", DRV8833_NSLEEP);

    // 2. 强制电机控制引脚为低电平（防止上电误动作）
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << M1_IN1_GPIO) | (1ULL << M1_IN2_GPIO) |
                        (1ULL << M2_IN3_GPIO) | (1ULL << M2_IN4_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(M1_IN1_GPIO, 0);
    gpio_set_level(M1_IN2_GPIO, 0);
    gpio_set_level(M2_IN3_GPIO, 0);
    gpio_set_level(M2_IN4_GPIO, 0);

    // 3. 配置 LEDC 定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 4. 配置四个 PWM 通道
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .hpoint = 0,
        .flags.output_invert = 0};

    ch_conf.gpio_num = M1_IN1_GPIO;
    ch_conf.channel = CH_M1_FWD;
    ch_conf.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ch_conf.gpio_num = M1_IN2_GPIO;
    ch_conf.channel = CH_M1_REV;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ch_conf.gpio_num = M2_IN3_GPIO;
    ch_conf.channel = CH_M2_FWD;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ch_conf.gpio_num = M2_IN4_GPIO;
    ch_conf.channel = CH_M2_REV;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    // 5. 确保停止
    motor_set_speed(0, 0);
    ESP_LOGI(TAG, "Motor driver initialized");
}

void motor_set_speed(float left_percent, float right_percent)
{

    // 限制范围
    if (left_percent > 100.0f)
        left_percent = 100.0f;
    if (left_percent < -100.0f)
        left_percent = -100.0f;
    if (right_percent > 100.0f)
        right_percent = 100.0f;
    if (right_percent < -100.0f)
        right_percent = -100.0f;

    int left_duty = percent_to_duty(fabsf(left_percent));
    int right_duty = percent_to_duty(fabsf(right_percent));

    // 左电机
    if (left_percent > 0)
    {
        ledc_set_duty(LEDC_MODE, CH_M1_FWD, left_duty);
        ledc_set_duty(LEDC_MODE, CH_M1_REV, 0);
    }
    else if (left_percent < 0)
    {
        ledc_set_duty(LEDC_MODE, CH_M1_FWD, 0);
        ledc_set_duty(LEDC_MODE, CH_M1_REV, left_duty);
    }
    else
    {
        ledc_set_duty(LEDC_MODE, CH_M1_FWD, 0);
        ledc_set_duty(LEDC_MODE, CH_M1_REV, 0);
    }
    ledc_update_duty(LEDC_MODE, CH_M1_FWD);
    ledc_update_duty(LEDC_MODE, CH_M1_REV);

    // 右电机
    if (right_percent > 0)
    {
        ledc_set_duty(LEDC_MODE, CH_M2_FWD, right_duty);
        ledc_set_duty(LEDC_MODE, CH_M2_REV, 0);
    }
    else if (right_percent < 0)
    {
        ledc_set_duty(LEDC_MODE, CH_M2_FWD, 0);
        ledc_set_duty(LEDC_MODE, CH_M2_REV, right_duty);
    }
    else
    {
        ledc_set_duty(LEDC_MODE, CH_M2_FWD, 0);
        ledc_set_duty(LEDC_MODE, CH_M2_REV, 0);
    }
    ledc_update_duty(LEDC_MODE, CH_M2_FWD);
    ledc_update_duty(LEDC_MODE, CH_M2_REV);
}
