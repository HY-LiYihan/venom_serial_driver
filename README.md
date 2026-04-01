# venom_serial_driver

DJI C 板串口通信驱动，用于 RoboMaster 机器人视觉系统与电控板之间的数据交互。

## 功能特性

- 双向二进制协议实现（基于 `vision_navigation_protocol.h`）
- CRC16 数据校验
- 滑动窗口帧解析
- 超时保护机制
- ROS 2 话题接口（自定义消息：`AutoAimCmd`、`RobotStatus`、`GameStatus`）

## 依赖

- ROS 2 Humble
- Python 3
- pyserial

## 安装

```bash
cd ~/venom_ws
colcon build --packages-select venom_serial_driver
source install/setup.bash
```

## 启动

```bash
ros2 launch venom_serial_driver serial_driver.launch.py
```

自定义串口参数：

```bash
ros2 launch venom_serial_driver serial_driver.launch.py \
  port_name:=/dev/ttyUSB0 \
  baud_rate:=921600
```

## ROS 2 话题

| 方向 | 话题 | 消息类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令（`linear.x/y` + `angular.z`，其中 `angular.z` 映射到协议 `linear_z` 作为底盘运动角速度，非自转） |
| 订阅 | `/auto_aim` | `AutoAimCmd` | 云台角度指令 + 自瞄状态，角度单位为 `rad` |
| 发布 | `/robot_status` | `RobotStatus` | 底盘速度 + 云台角度反馈，角度单位为 `rad` |
| 发布 | `/game_status` | `GameStatus` | 比赛状态、血量、热量等 |

## 参数配置

编辑 `config/serial_params.yaml`：

```yaml
serial_node:
  ros__parameters:
    port_name: "/dev/ttyUSB0"  # 串口设备
    baud_rate: 921600           # 波特率
    timeout: 0.1                # 读取超时（秒）
    loop_rate: 50               # 定时器频率（Hz）
    cmd_vel_topic: "/cmd_vel"   # 底盘速度指令订阅话题
    auto_aim_topic: "/auto_aim" # 自瞄指令订阅话题
```

## 文档

- [通信协议（中文）](docs/protocol_zh.md)
- [Communication Protocol (English)](docs/protocol.md)

## 测试

```bash
# 回环与 CRC 验证（无需硬件）
python3 test/test_loopback.py

# 实时状态监控（需要串口连接）
python3 test/test_monitor.py --port /dev/ttyUSB0

# 协议监视与测试发送（yaw/pitch 使用 rad）
python3 test/test_protocol_monitor.py --port /dev/ttyUSB0 --send-test --yaw 0.1 --pitch -0.05

# 硬件运动测试
python3 test/test_hardware.py --port /dev/ttyUSB0 --test yaw
python3 test/test_hardware.py --port /dev/ttyUSB0 --test chassis
```

## 故障排查

1. **串口权限问题**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **检查串口设备**
   ```bash
   ls /dev/ttyUSB*
   ```

3. **开启调试日志**
   ```bash
   ros2 run venom_serial_driver serial_node --ros-args --log-level debug
   ```

## License

MIT
