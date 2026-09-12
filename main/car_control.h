// car_control.h
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------
 * 统一速度单位：m/s
 * 上层协议（BLE/Web）仍然发送 ±100 的归一化值，
 * 由各自的转换点负责换算为 m/s：
 *     速度(m/s) = 上层值 / 100.0f * CAR_MAX_SPEED_MS
 * ------------------------------------------------------------------ */
#define CAR_MAX_SPEED_MS   1.0f   /* 满油门对应的线速度（m/s），按需修改 */

typedef struct {
    int   stop;             /* 1: 停止 */
    float target_speed;     /* 目标线速度 (m/s)，范围 ±CAR_MAX_SPEED_MS */
    float target_turn;      /* 目标偏航角速度 (°/s) */

    /* 速度环 PI 参数：输入误差单位 m/s，输出期望俯仰角（度）
     *   kp: 度/(m/s)
     *   ki: 度/(m/s·s)
     *   kd: 暂未使用（速度环用 PI 即可） */
    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;

    float turn_gain;        /* 保留字段（当前未使用） */
} car_control_params_t;

void car_control_init(void);
void car_control_update_params(const car_control_params_t *params);
void car_control_get_params(car_control_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H