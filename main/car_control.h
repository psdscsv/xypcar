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
#define CAR_MAX_SPEED_MS   10.0f   /* 满油门对应的线速度（m/s），按需修改 */

typedef struct {
    int   stop;             /* 1: 停止 */
    float target_speed;     /* 目标线速度 (m/s)，范围 ±CAR_MAX_SPEED_MS */
    float target_turn;      /* 目标偏航角速度 (°/s) */

    float pid_kp;
    float pid_ki;
    float pid_kd;

    float pid_flag;        
} car_control_params_t;

void car_control_init(void);
void car_control_update_params(const car_control_params_t *params);
void car_control_get_params(car_control_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H