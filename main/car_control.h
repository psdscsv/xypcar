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
        int stop;
        float target_speed; // -100 ~ 100
        float target_turn;  // -100 ~ 100 → 将被转换为角速度

        float kp_roll;
        float kp_pitch;

        float speed_pid_kp;
        float speed_pid_ki;
        float speed_pid_kd;

        float turn_gain;         // 保留但不再使用（兼容旧版）
        float max_linear_speed;  // 对应100%目标线速度的m/s值
        float max_angular_speed; // 新增：对应100%目标转向的角速度(°/s)
    } car_control_params_t;

    void car_control_init(void);
    void car_control_update_params(const car_control_params_t *params);
    void car_control_get_params(car_control_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H