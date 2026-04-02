# 通信协议说明

NUC（上位机）与 DJI C 板（下位机）之间的串口二进制通信协议。

参考来源：C 板固件中的 `vision_navigation_protocol.h` / `vision_navigation_protocol.c`。

## 帧格式

两个方向使用相同的 SOF 字节（`0xA5`）和相同的 4 字节帧头格式：

```
[SOF(1)] [data_length(2, 小端)] [cmd_id(1)] [payload(N)] [CRC16(2, 小端)]
```

| 字段 | 大小 | 说明 |
|---|---|---|
| `SOF` | 1 字节 | `0xA5`，收发两个方向相同 |
| `data_length` | 2 字节 | payload 长度（小端序） |
| `cmd_id` | 1 字节 | `0x02` = NUC→C 板控制帧；`0x01` = C 板→NUC 反馈帧 |
| `payload` | N 字节 | 见下表 |
| `CRC16` | 2 字节 | 对帧头+payload 计算，小端序存储 |

---

## 1. NUC → C 板 控制帧

**cmd_id：** `0x02`
**payload 类型：** `Vision_navigation_ctrl_payload_t`
**payload 大小：** 33 字节

坐标系约定（底盘运动参考坐标系，以雷达安装位置为基准）：
- `x` 轴正方向：车体正前方
- `y` 轴正方向：车体左侧
- `z` 轴正方向：车体上方（右手系）

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `status_flags` | `uint8` | bit0 = 检测到目标，bit1 = 正在追踪，bit2 = 允许开火 |
| 1–4 | `chassis_vx` | `float32` | 底盘 `x` 方向线速度（m/s） |
| 5–8 | `chassis_vy` | `float32` | 底盘 `y` 方向线速度（m/s） |
| 9–12 | `chassis_motion_wz` | `float32` | 底盘运动角速度（非自转，rad/s） |
| 13–16 | `chassis_world_wz` | `float32` | 底盘旋转角速度（世界坐标系基准，rad/s），与运动控制无关 |
| 17–20 | `angular_y` | `float32` | 云台 pitch 角度指令（rad，单圈值，非多圈，范围 `[-pi, pi)`） |
| 21–24 | `angular_z` | `float32` | 云台 yaw 角度指令（rad，单圈值，非多圈，范围 `[-pi, pi)`） |
| 25–28 | `distance` | `float32` | 保留 |
| 29–30 | `frame_x` | `uint16` | 目标像素 x 坐标 |
| 31–32 | `frame_y` | `uint16` | 目标像素 y 坐标 |

**完整帧长度：** 4（帧头）+ 33（payload）+ 2（CRC16）= **39 字节**

---

## 2. C 板 → NUC 反馈帧

**cmd_id：** `0x01`
**payload 类型：** `Vision_navigation_feedback_payload_t`
**payload 大小：** 72 字节

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0–3 | `timestamp_us` | `uint32` | C 板时间戳（微秒） |
| 4–7 | `chassis_vx` | `float32` | 底盘 `x` 方向线速度（m/s） |
| 8–11 | `chassis_vy` | `float32` | 底盘 `y` 方向线速度（m/s） |
| 12–15 | `chassis_motion_wz` | `float32` | 底盘运动角速度（非自转，rad/s） |
| 16–19 | `chassis_world_wz` | `float32` | 底盘旋转角速度（世界坐标系基准，rad/s），与运动控制无关 |
| 20–23 | `angular_y` | `float32` | 云台 pitch 角度（rad，单圈值，非多圈，范围 `[-pi, pi)`） |
| 24–27 | `angular_z` | `float32` | 云台 yaw 角度（rad，单圈值，非多圈，范围 `[-pi, pi)`） |
| 28–31 | `angular_y_speed` | `float32` | 云台 pitch 角速度（rad/s） |
| 32–35 | `angular_z_speed` | `float32` | 云台 yaw 角速度（rad/s） |
| 36–39 | `distance` | `float32` | 保留 |
| 40 | `game_progress` | `uint8` | 低 4 位：比赛阶段（0=未开始，1=准备，2=自检，3=倒计时，4=比赛中，5=结算） |
| 41–42 | `stage_remain_time` | `uint16` | 当前阶段剩余时间（s）。当前实现：始终为 0 |
| 43 | `center_outpost_occupancy` | `uint8` | 低 2 位：0=未占领，1=己方，2=对方，3=双方。当前实现：始终为 0 |
| 44–45 | `current_HP` | `uint16` | 机器人当前血量 |
| 46–47 | `maximum_HP` | `uint16` | 机器人血量上限 |
| 48–49 | `shooter_barrel_heat_limit` | `uint16` | 射击热量上限。当前实现：始终为 0 |
| 50 | `power_management_output` | `uint8` | 电源输出标志（bit0=云台，bit1=底盘，bit2=射击）。当前实现：始终为 0 |
| 51–52 | `shooter_17mm_barrel_heat` | `uint16` | 17mm 发射热量。当前实现：始终为 0 |
| 53–54 | `shooter_42mm_barrel_heat` | `uint16` | 42mm 发射热量。当前实现：始终为 0 |
| 55 | `armor_id_and_reason` | `uint8` | 低 4 位 = 扣血装甲 ID；高 4 位 = 扣血原因（0=弹丸，1=离线，5=碰撞）。当前实现：始终为 0 |
| 56–59 | `launching_frequency` | `float32` | 弹丸发射频率（Hz）。当前实现：始终为 0 |
| 60–63 | `initial_speed` | `float32` | 弹丸初速度（m/s）。当前实现：始终为 0 |
| 64–65 | `projectile_allowance_17mm` | `uint16` | 17mm 剩余允许发弹量。当前实现：始终为 0 |
| 66–67 | `projectile_allowance_42mm` | `uint16` | 42mm 剩余允许发弹量。当前实现：始终为 0 |
| 68–71 | `rfid_status` | `uint32` | RFID 增益点状态位掩码（bit 0–31 对应各增益点 ID）。当前实现：始终为 0 |

**完整帧长度：** 4（帧头）+ 72（payload）+ 2（CRC16）= **78 字节**

---

## CRC16

算法：CRC16/IBM（与 DJI RoboMaster 裁判系统相同查找表）。
初始值：`0xFFFF`。
计算范围：CRC 字段之前的所有字节。
存储方式：小端序。
