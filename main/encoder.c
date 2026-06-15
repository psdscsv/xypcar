#include "encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Encoder";

// 引脚定义
#define ENC_LEFT_A_GPIO  8
#define ENC_LEFT_B_GPIO  9
#define ENC_RIGHT_A_GPIO 10
#define ENC_RIGHT_B_GPIO 11

// 编码器原始PPR（每转脉冲数，未经倍频）
#define PPR_RAW 69
// 轮子半径（米）
#define WHEEL_RADIUS 0.066f
#define TWO_PI 6.283185307f

// 全局脉冲计数（中断中修改，使用临界区保护）
static volatile int s_left_pulse = 0;
static volatile int s_right_pulse = 0;

// 上次读取的脉冲值和时间（用于速度计算）
static int s_last_left_pulse = 0;
static int s_last_right_pulse = 0;
static uint32_t s_last_time_ms = 0;

// 临界区保护变量（用于中断和任务间的原子访问）
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// 左电机正交解码状态机
// 记录上一次AB两相的状态（0~3）
static volatile uint8_t s_left_last_state = 0;
static volatile uint8_t s_right_last_state = 0;

// 正交解码增量表（上一个状态，当前状态）-> 脉冲变化值
// 根据标准正交编码器规则：顺时针旋转时状态变化顺序 0->1->3->2->0...
// 这里使用常见索引表，若方向反了可交换A/B或修改表格
static const int8_t s_quadrature_table[4][4] = {
    {0, 1, -1, 0}, // 从状态0到0,1,2,3
    {-1, 0, 0, 1}, // 从状态1
    {1, 0, 0, -1}, // 从状态2
    {0, -1, 1, 0}  // 从状态3
};

// 通用中断服务：读取当前AB电平，更新脉冲计数
static void IRAM_ATTR encoder_isr(gpio_num_t gpio_a, gpio_num_t gpio_b,
                                  volatile int *pulse_counter, volatile uint8_t *last_state)
{
    uint8_t a = gpio_get_level(gpio_a);
    uint8_t b = gpio_get_level(gpio_b);
    uint8_t curr_state = (a << 1) | b; // 组合成2位状态值

    // 查表得到增量
    int8_t inc = s_quadrature_table[*last_state][curr_state];
    if (inc != 0)
    {
        portENTER_CRITICAL_ISR(&s_mux);
        *pulse_counter += inc;
        portEXIT_CRITICAL_ISR(&s_mux);
    }
    *last_state = curr_state;
}

// 左电机中断（A相）
static void IRAM_ATTR left_isr_handler(void *arg)
{
    encoder_isr(ENC_LEFT_A_GPIO, ENC_LEFT_B_GPIO, &s_left_pulse, &s_left_last_state);
}

// 右电机中断（A相）
static void IRAM_ATTR right_isr_handler(void *arg)
{
    encoder_isr(ENC_RIGHT_A_GPIO, ENC_RIGHT_B_GPIO, &s_right_pulse, &s_right_last_state);
}

bool encoder_init(void)
{
    // 配置GPIO输入，内部上拉
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENC_LEFT_A_GPIO) | (1ULL << ENC_LEFT_B_GPIO) |
                        (1ULL << ENC_RIGHT_A_GPIO) | (1ULL << ENC_RIGHT_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 设置A、B两相均为任意边沿中断（上升沿+下降沿）
    gpio_set_intr_type(ENC_LEFT_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENC_LEFT_B_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENC_RIGHT_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENC_RIGHT_B_GPIO, GPIO_INTR_ANYEDGE);

    // 安装GPIO中断服务（全局一次）
    static bool isr_installed = false;
    if (!isr_installed)
    {
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        isr_installed = true;
    }

    // 注册中断处理函数
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_LEFT_A_GPIO, left_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_LEFT_B_GPIO, left_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_RIGHT_A_GPIO, right_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_RIGHT_B_GPIO, right_isr_handler, NULL));

    // 读取初始电平，初始化状态机
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

void encoder_get_speed(float *left_speed_ms, float *right_speed_ms)
{
    if (!left_speed_ms || !right_speed_ms)
        return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - s_last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f)
    { // 防止异常间隔
        *left_speed_ms = 0;
        *right_speed_ms = 0;
        s_last_time_ms = now_ms;
        return;
    }

    int left_pulse, right_pulse;
    encoder_get_pulse(&left_pulse, &right_pulse);

    // 脉冲增量
    int left_delta = left_pulse - s_last_left_pulse;
    int right_delta = right_pulse - s_last_right_pulse;

    // 转换为线速度（米/秒）
    // 注意：每转脉冲数 = PPR_RAW * 4（因为4倍频）
    float pulses_per_rev = PPR_RAW * 4.0f;
    float wheel_circumference = TWO_PI * WHEEL_RADIUS;
    *left_speed_ms = (left_delta / pulses_per_rev) * wheel_circumference / dt;
    *right_speed_ms = (right_delta / pulses_per_rev) * wheel_circumference / dt;

    // 更新缓存
    s_last_left_pulse = left_pulse;
    s_last_right_pulse = right_pulse;
    s_last_time_ms = now_ms;
}
void encoder_get_speed_pps(float *left_pps, float *right_pps) {
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now_ms - s_last_time_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) { *left_pps = *right_pps = 0; return; }
    int left_pulse, right_pulse;
    encoder_get_pulse(&left_pulse, &right_pulse);
    float pulses_per_sec_left = (left_pulse - s_last_left_pulse) / dt;
    float pulses_per_sec_right = (right_pulse - s_last_right_pulse) / dt;
    *left_pps = pulses_per_sec_left;
    *right_pps = pulses_per_sec_right;
    s_last_left_pulse = left_pulse;
    s_last_right_pulse = right_pulse;
    s_last_time_ms = now_ms;
}