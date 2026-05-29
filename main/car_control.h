// car_control.h
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        // 实时控制指令
        int stop;           // 0: 正常，1: 紧急停止
        float target_speed; // -100 ~ 100
        float target_turn;  // -100 ~ 100

        // 姿态内环 PID 参数
        float kp_roll;
        float kp_pitch;

        // 速度外环 PID 参数
        float speed_pid_kp;
        float speed_pid_ki;
        float speed_pid_kd;

        // 其他增益
        float turn_gain;        // 转向灵敏度
        float max_linear_speed; // 最大线速度(m/s)，对应100%目标速度
        float speed_pitch_gain; // 已弃用，保留兼容
    } car_control_params_t;

    void car_control_init(void);
    void car_control_update_params(const car_control_params_t *params);
    void car_control_get_params(car_control_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H