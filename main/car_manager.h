// car_manager.h
#ifndef CAR_MANAGER_H
#define CAR_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化小车遥控管理模块
     *        - 注册 HTTP 处理程序到 WiFi 管理器的 Web 服务器
     *        - 启动小车控制相关功能
     */
    void car_manager_init(void);

#ifdef __cplusplus
}
#endif

#endif // CAR_MANAGER_H