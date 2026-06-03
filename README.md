# Butterfly WIFI Drone 

本方案基于：

- ESP32-S3 Super Mini
- MPU6050 姿态传感器
- 2S 电池
- ARMJSHU_AJ33 降压模块，输出 5V
- 双舵机扑翼机构
- WiFi 控制网页



## 推荐开发顺序

1. 先看 `docs/wiring.md` 完成接线。
2. 最后烧录 `arduino/dual_servo_wifi_control/dual_servo_wifi_control.ino`，用中文网页控制扑翼。

## 默认引脚

| 功能 | ESP32-S3 引脚 |
|---|---|
| MPU6050 SCL | GPIO5 |
| MPU6050 SDA | GPIO6 |
| 左舵机 Signal | GPIO4 |
| 右舵机 Signal | GPIO7 |

如果你的板子没有 GPIO7 可用，可以把代码中的 `RIGHT_SERVO_PIN` 改成 GPIO1 / GPIO2 / GPIO3 / GPIO8 中的空闲脚。

## Arduino 依赖库

在 Arduino IDE 库管理器中安装：

- `MPU6050_light`
- `ESP32Servo`

WiFi 网页控制版使用 ESP32 Arduino 内置库：

- `WiFi.h`
- `WebServer.h`

## WiFi 中文控制版默认热点

烧录 `dual_servo_wifi_control.ino` 后，ESP32 会创建热点：

- SSID: `ButterflyDrone`
- Password: `12345678`

连接后浏览器打开：

```text
http://192.168.4.1
```

可控制：

- 启动 / 停止
- 模式：STOP / TAKEOFF / CRUISE / CORRECT / LAND
- 大箭头快速控制：▲ 加速、▼ 减速、◀ 左转、▶ 右转
- 方向回正
- 扑翼幅度
- 扑翼速度
- 左右相位差
- 左右舵机是否反向

## 重要供电提醒

舵机不要从 ESP32 的 3.3V 取电。建议：

- ARMJSHU_AJ33 5V 输出供 ESP32 VIN/5V
- ARMJSHU_AJ33 5V 输出同时供舵机 VCC
- ESP32、MPU6050、舵机、降压模块全部共地
- 舵机电源端并联 470uF 到 1000uF 电解电容

## 安全调试建议

第一次烧录后不要装翅膀，先让舵机空载动作，确认角度、方向和供电稳定后再装连杆和翅膀。
