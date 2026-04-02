# 话题输入与输出说明（中文）

本文档说明 `venom_serial_driver/serial_node` 的 ROS 2 话题接口，用于联调和排障。

## 输入话题（订阅）

### 1) `/cmd_vel` (`geometry_msgs/Twist`)

用于底盘运动控制。当前仅使用以下字段：

| 字段 | 是否使用 | 说明 |
|---|---|---|
| `linear.x` | 是 | 底盘 `x` 方向线速度 |
| `linear.y` | 是 | 底盘 `y` 方向线速度 |
| `angular.z` | 是 | 底盘运动角速度（非自转） |
| `linear.z` | 否 | 非法输入（非 0 时会打印 `warn`） |
| `angular.x` | 否 | 非法输入（非 0 时会打印 `warn`） |
| `angular.y` | 否 | 非法输入（非 0 时会打印 `warn`） |

### 2) `/auto_aim` (`AutoAimCmd`)

用于云台角度与自瞄状态控制。

| 字段 | 说明 |
|---|---|
| `pitch`, `yaw` | 云台角度指令（单位 `rad`） |
| `detected`, `tracking`, `fire` | 目标检测/跟踪/开火标志 |
| `distance` | 目标距离 |
| `proj_x`, `proj_y` | 图像投影点坐标 |

说明：
- 若超出 `vision_timeout` 未收到新 `auto_aim`，节点会自动下发零值云台角度与空状态。

## 输出话题（发布）

### 1) `/venom_cmd` (`std_msgs/String`)

调试话题。每次下发串口控制帧时都会发布一条文本，包含：
- 触发源（`cmd_vel` 或 `auto_aim`）
- 关键控制字段（底盘、云台、flags、frame）
- 原始十六进制帧（`raw=...`）

适合用 `ros2 topic echo /venom_cmd` 快速确认“到底发了什么给 C 板”。

### 2) `/robot_status` (`RobotStatus`)

底盘与云台状态反馈，来自 C 板状态帧解析结果。

### 3) `/game_status` (`GameStatus`)

比赛状态、血量、热量、弹丸等信息，来自 C 板状态帧解析结果。

## 推荐联调顺序

1. `ros2 topic echo /venom_cmd`：确认下发指令是否正确。
2. `ros2 topic echo /robot_status`：确认回传底盘/云台状态。
3. `ros2 topic echo /game_status`：确认比赛与裁判信息字段。

## 相关参数

`serial_node` 常用话题参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `cmd_vel_topic` | `/cmd_vel` | 底盘命令订阅话题 |
| `auto_aim_topic` | `/auto_aim` | 自瞄命令订阅话题 |
| `venom_cmd_topic` | `/venom_cmd` | 调试命令发布话题 |
| `robot_status_topic` | `/robot_status` | 机器人状态发布话题 |
| `game_status_topic` | `/game_status` | 比赛状态发布话题 |
