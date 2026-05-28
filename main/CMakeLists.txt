idf_component_register(SRCS "ble_control.c" "attitude_control.c" "mpu6050.c" "main.c"
                            "motor_control.c"
                            "wifi_manager.c"
                            "car_manager.c"
                            "mpu6050.c"
                            "attitude_control.c"
                            "ble_control.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES
                        esp_wifi       # wifi组件依赖
                        nvs_flash
                        esp_netif
                        lwip
                        esp_event      # 配网组件
                        esp_http_server
                        driver
                        bt)
