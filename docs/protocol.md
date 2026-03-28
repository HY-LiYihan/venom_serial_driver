# Serial Communication Protocol

Binary protocol between NUC (upper computer) and DJI C-board (lower computer).

Reference: `vision_navigation_protocol.h` / `vision_navigation_protocol.c` in the C-board firmware.

## Frame Structure

Both directions use the same SOF byte (`0xA5`) and the same 4-byte header format:

```
[SOF(1)] [data_length(2, little-endian)] [cmd_id(1)] [payload(N)] [CRC16(2, little-endian)]
```

| Field | Size | Description |
|---|---|---|
| `SOF` | 1 byte | `0xA5` — both TX and RX directions |
| `data_length` | 2 bytes | Payload length in bytes (little-endian) |
| `cmd_id` | 1 byte | `0x02` = NUC→C-board control; `0x01` = C-board→NUC feedback |
| `payload` | N bytes | See tables below |
| `CRC16` | 2 bytes | CRC16 over all preceding bytes (little-endian) |

---

## 1. NUC → C-board Control Frame

**cmd_id:** `0x02`
**Payload type:** `Vision_navigation_ctrl_payload_t`
**Payload size:** 33 bytes

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | `status_flags` | `uint8` | bit0 = armor_detected, bit1 = tracking_state, bit2 = fire |
| 1–4 | `linear_x` | `float32` | Chassis forward/backward velocity (m/s) |
| 5–8 | `linear_y` | `float32` | Chassis left/right velocity (m/s) |
| 9–12 | `linear_z` | `float32` | Reserved, set to 0 |
| 13–16 | `angular_x` | `float32` | Chassis rotation angular velocity (rad/s) |
| 17–20 | `angular_y` | `float32` | Gimbal pitch angle command (deg) |
| 21–24 | `angular_z` | `float32` | Gimbal yaw angle command (deg) |
| 25–28 | `distance` | `float32` | Reserved |
| 29–30 | `frame_x` | `uint16` | Target pixel x coordinate |
| 31–32 | `frame_y` | `uint16` | Target pixel y coordinate |

**Total frame length:** 4 (header) + 33 (payload) + 2 (CRC16) = **39 bytes**

---

## 2. C-board → NUC Feedback Frame

**cmd_id:** `0x01`
**Payload type:** `Vision_navigation_feedback_payload_t`
**Payload size:** 72 bytes

| Offset | Field | Type | Description |
|---|---|---|---|
| 0–3 | `timestamp_us` | `uint32` | C-board timestamp (microseconds) |
| 4–7 | `linear_x` | `float32` | Chassis forward/backward velocity (m/s) |
| 8–11 | `linear_y` | `float32` | Chassis left/right velocity (m/s) |
| 12–15 | `linear_z` | `float32` | Reserved |
| 16–19 | `gyro_wz` | `float32` | Chassis rotation angular velocity (rad/s) |
| 20–23 | `angular_y` | `float32` | Gimbal pitch angle (deg) |
| 24–27 | `angular_z` | `float32` | Gimbal yaw angle (deg) |
| 28–31 | `angular_y_speed` | `float32` | Gimbal pitch angular velocity (rad/s) |
| 32–35 | `angular_z_speed` | `float32` | Gimbal yaw angular velocity (rad/s) |
| 36–39 | `distance` | `float32` | Reserved |
| 40 | `game_progress` | `uint8` | Low 4 bits: game stage (0=not started … 4=in progress … 5=ended) |
| 41–42 | `stage_remain_time` | `uint16` | Remaining time in current stage (s). Current implementation: always 0 |
| 43 | `center_outpost_occupancy` | `uint8` | Low 2 bits: 0=none, 1=friendly, 2=enemy, 3=both. Current implementation: always 0 |
| 44–45 | `current_HP` | `uint16` | Robot current HP |
| 46–47 | `maximum_HP` | `uint16` | Robot maximum HP |
| 48–49 | `shooter_barrel_heat_limit` | `uint16` | Barrel heat limit. Current implementation: always 0 |
| 50 | `power_management_output` | `uint8` | Power output flags (bit0=gimbal, bit1=chassis, bit2=shooter). Current implementation: always 0 |
| 51–52 | `shooter_17mm_barrel_heat` | `uint16` | 17mm barrel heat. Current implementation: always 0 |
| 53–54 | `shooter_42mm_barrel_heat` | `uint16` | 42mm barrel heat. Current implementation: always 0 |
| 55 | `armor_id_and_reason` | `uint8` | Low 4 bits = armor_id (hit armor plate ID); high 4 bits = HP_deduction_reason (0=projectile, 1=offline, 5=collision). Current implementation: always 0 |
| 56–59 | `launching_frequency` | `float32` | Projectile launch frequency (Hz). Current implementation: always 0 |
| 60–63 | `initial_speed` | `float32` | Projectile initial speed (m/s). Current implementation: always 0 |
| 64–65 | `projectile_allowance_17mm` | `uint16` | Remaining 17mm projectile allowance. Current implementation: always 0 |
| 66–67 | `projectile_allowance_42mm` | `uint16` | Remaining 42mm projectile allowance. Current implementation: always 0 |
| 68–71 | `rfid_status` | `uint32` | RFID zone status bitmask (bits 0–31 = zone IDs). Current implementation: always 0 |

**Total frame length:** 4 (header) + 72 (payload) + 2 (CRC16) = **78 bytes**

---

## CRC16

Algorithm: CRC16/IBM (same table as DJI RoboMaster referee system).
Initial value: `0xFFFF`.
Covers all bytes before the CRC field.
Stored little-endian.
