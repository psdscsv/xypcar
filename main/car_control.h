// car_control.h
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int stop;               // 1: 停止
    float target_speed;     // 目标线速度，单位 0.01 m/s (100 = 1.00 m/s)
    float target_turn;      // 转向值 -100 ~ 100

    float turn_gain;        // 转向增益
    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
} car_control_params_t;

void car_control_init(void);
void car_control_update_params(const car_control_params_t *params);
void car_control_get_params(car_control_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H