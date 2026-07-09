# 校园跑足球机器人

基于 ESP32-S3 的两轮球型小车，集成了**MPU6050 姿态传感器**、**正交编码器** 和 **DRV8833 电机驱动**，支持 **WiFi 配网**、**Web 遥控**、**BLE 控制**，实现远程机器人控制。

---

## ✨ 特性

- ⚖️ **自平衡控制**  
  采用互补滤波融合 MPU6050 加速度/陀螺仪数据，级联 PID（速度外环 + 姿态内环）实现稳定运动。

- 📡 **多模式通信**  
  - **WiFi 配网**：开机自动尝试连接已保存的 WiFi，失败则开启 AP 热点（SSID: `灵眸智译-192.168.4.1`）并提供 Web 配置页面。  
  - **Web 遥控**：访问 `192.168.4.1/control` 使用摇杆控制小车前进/后退/转向。  
  - **BLE 遥控**：通过自定义 GATT 服务（UUID: 0xFFE0/0xFFE1）接收 25 字节控制包。  
  - **UART 指令**：通过串口（GPIO17 TX，GPIO38 RX）发送控制命令。

- 🔍 **自动发现**  
  手机 App 自动扫描连接小车（app正在开发中）。

## 📦 依赖组件

- **ESP-IDF** v5.0 及以上（推荐 v5.5）
- 外部组件（已在 `idf_component.yml` 或 `CMakeLists.txt` 中声明）：
  - 标准 IDF 组件：`bt`, `esp_wifi`, `nvs_flash`, `esp_netif`, `lwip`, `esp_event`, `esp_http_server`, `driver`, `spi_flash`

---

## 🚀 构建与烧录

### 1. 克隆项目
```bash
git clone https://github.com/psdscsv/xypcar.git
cd xypcar
```

### 2. 安装 ESP-IDF 环境
确保已安装 [ESP-IDF 工具链](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/index.html)，并设置环境变量。

### 3. 获取组件
一般构建项目代码时会自动下载

### 4. 配置与编译
```bash
idf.py set-target esp32s3
idf.py fullclean
idf.py build
```

### 5. 烧录与监控
```bash
idf.py -p /dev/ttyUSB0 flash monitor   # Windows 下端口为 COMx
```

---

## 📱 使用说明
>本人写了安卓app用于遥控小车，等做得差不多了就会发布
### WiFi 配网（使用web遥控的前提，如果不用就不管）
- 首次上电，设备会尝试连接上次保存的 WiFi。若失败，自动开启 AP 热点 `ESP32-CAM`（密码123456）。
- 手机连接该热点，打开浏览器访问 `http://192.168.4.1`，即可看到遥控页面。
- 输入目标 WiFi 的 SSID 和密码，提交后设备保存并重启，之后小车将自动连接该 WiFi。

### Web 遥控
- 设备连接 WiFi 后，在浏览器（手机或电脑）访问设备 IP 地址（可通过路由器查看，如果是手机连接设备的wifi，ip地址就是192.168.4.1），即可看到单摇杆遥控页面。
- 拖动摇杆控制小车前进/后退/转向，指令会通过 HTTP GET 请求发送至 `/api/joystick`。

### BLE 控制
- 使用 BLE 扫描工具搜索名称为 `ESP32_Car` 的设备。
- 连接后，向特征 UUID `0xFFE1`（服务 UUID `0xFFE0`）写入 25 字节的控制包（格式见下文）。
- 小车将根据包中的速度、转向、PID 参数等实时响应。


### 串口控制
- 波特率 115200，数据位 8，无校验，1 停止位。
- 可通过 `uart_control_send()` 发送调试数据，或接收外部指令（当前未实现具体协议，可作为扩展接口）。

---

## 📦 通信协议

### BLE 控制包（25 字节）

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | 0xA5 | 包头 |
| 1-2 | 2 | speed | 目标速度（int16，单位 0.01 m/s） |
| 3-4 | 2 | turn | 转向值（int16，-100~100） |
| 5-6 | 2 | stop | 0/1 停止标志 |
| 7-10 | 4 | turn_gain | 转向增益（float） |
| 11-14 | 4 | speed_kp | 速度外环 Kp（float） |
| 15-18 | 4 | speed_ki | 速度外环 Ki（float） |
| 19-22 | 4 | speed_kd | 速度外环 Kd（float） |
| 23 | 1 | checksum | 字节 1~22 累加和 |
| 24 | 1 | 0x5A | 包尾 |
---

## 🧪 调试与调参

- 串口监视器会输出姿态角度、速度、PID 输出等关键信息，方便调试。
- 支持通过 BLE 动态修改 PID 参数（见 `attitude_set_speed_pid` 等函数）。

---

## 🤝 贡献与许可

欢迎提交 Issue 和 Pull Request。

许可证详情见 [LICENSE](LICENSE) 文件。

---

**祝您玩得开心！** 如有疑问，请提 Issue 或在Bilibili联系我（用户名字和github相同）。