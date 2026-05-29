idf_component_register(SRCS "attitude_control.c"
                            "ble_control.c"
                            "car_control.c"
                            "encoder.c"
                            "main.c"
                            "motor_control.c"
                            "mpu6050.c"
                            "wifi_manager.c"
                            "web_control.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES
                        esp_wifi
                        nvs_flash
                        esp_netif
                        lwip
                        esp_event
                        esp_http_server
                        driver
                        bt
                        )