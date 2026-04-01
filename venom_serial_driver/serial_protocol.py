"""Binary protocol implementation for DJI C-board serial communication.

Implements the frame packing and unpacking logic based on
vision_navigation_protocol.h from the C-board firmware.
"""
import struct
if __package__:
    from venom_serial_driver.crc_utils import crc16, verify_crc16, append_crc16
else:
    from crc_utils import crc16, verify_crc16, append_crc16

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
SOF_TX = 0xA5  # NUC -> C-board (same SOF byte for both directions)
SOF_RX = 0xA5  # C-board -> NUC
CMD_ID_STATE = 0x01  # C-board feedback frame
CMD_ID_CTRL = 0x02   # Vision/navigation control frame


class RobotCtrlData:
    """Control data sent from NUC to C-board (Vision_navigation_ctrl_payload_t).

    Packed format: <B7f2H (33 bytes total).
    Field order matches the firmware struct exactly:
        status_flags  (uint8)
        linear_x      (float) - chassis forward/backward velocity (m/s)
        linear_y      (float) - chassis left/right velocity (m/s)
        linear_z      (float) - chassis motion angular velocity (non-spin, rad/s)
        chassis_wz    (float) - angular_x in firmware; reserved by current project convention
        angular_y     (float) - gimbal pitch angle (rad)
        angular_z     (float) - gimbal yaw angle (rad)
        distance      (float) - reserved
        frame_x       (uint16) - target pixel x
        frame_y       (uint16) - target pixel y
    """

    def __init__(self):
        self.flags = 0        # status_flags: bit0=detected, bit1=tracking, bit2=fire
        self.lx = 0.0         # linear_x
        self.ly = 0.0         # linear_y
        self.lz = 0.0         # linear_z: chassis motion angular velocity (non-spin, rad/s)
        self.chassis_wz = 0.0 # angular_x in firmware: reserved by current project convention
        self.ay = 0.0         # angular_y: gimbal pitch (rad)
        self.az = 0.0         # angular_z: gimbal yaw (rad)
        self.dist = 0.0       # distance (reserved)
        self.frame_x = 0      # target pixel x
        self.frame_y = 0      # target pixel y

    def pack(self) -> bytes:
        """Pack into little-endian byte stream. Format: <B7f2H."""
        return struct.pack('<B7f2H',
                          self.flags,
                          self.lx, self.ly, self.lz,
                          self.chassis_wz, self.ay, self.az,
                          self.dist,
                          self.frame_x, self.frame_y)


class RobotStateData:
    """State data received from C-board (Vision_navigation_feedback_payload_t).

    Packed format: <I9fBHB3HB2HB2f2HI (72 bytes total).
    """

    def __init__(self):
        self.timestamp_us = 0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.gyro_wz = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        self.angular_y_speed = 0.0
        self.angular_z_speed = 0.0
        self.distance = 0.0
        self.game_progress = 0
        self.stage_remain_time = 0
        self.center_outpost_occupancy = 0
        self.current_HP = 0
        self.maximum_HP = 0
        self.shooter_barrel_heat_limit = 0
        self.power_management = 0
        self.shooter_17mm_barrel_heat = 0
        self.shooter_42mm_barrel_heat = 0
        self.armor_id = 0
        self.HP_deduction_reason = 0
        self.launching_frequency = 0.0
        self.initial_speed = 0.0
        self.projectile_allowance_17mm = 0
        self.projectile_allowance_42mm = 0
        self.rfid_status = 0

    @staticmethod
    def unpack(data: bytes):
        """Unpack from little-endian byte stream (72-byte payload).

        Args:
            data: Raw payload bytes (must be at least 72 bytes).

        Returns:
            RobotStateData instance, or None if data is too short.
        """
        if len(data) < 72:
            return None

        state = RobotStateData()
        unpacked = struct.unpack('<I9fBHB3HB2HB2f2HI', data[:72])

        state.timestamp_us = unpacked[0]
        state.linear_x = unpacked[1]
        state.linear_y = unpacked[2]
        state.linear_z = unpacked[3]
        state.gyro_wz = unpacked[4]
        state.angular_y = unpacked[5]
        state.angular_z = unpacked[6]
        state.angular_y_speed = unpacked[7]
        state.angular_z_speed = unpacked[8]
        state.distance = unpacked[9]
        state.game_progress = unpacked[10] & 0x0F
        state.stage_remain_time = unpacked[11]
        state.center_outpost_occupancy = unpacked[12] & 0x03
        state.current_HP = unpacked[13]
        state.maximum_HP = unpacked[14]
        state.shooter_barrel_heat_limit = unpacked[15]
        state.power_management = unpacked[16]
        state.shooter_17mm_barrel_heat = unpacked[17]
        state.shooter_42mm_barrel_heat = unpacked[18]
        # armor_id_and_reason is a single byte: low 4 bits = armor_id, high 4 bits = reason
        state.armor_id = unpacked[19] & 0x0F
        state.HP_deduction_reason = (unpacked[19] >> 4) & 0x0F
        state.launching_frequency = unpacked[20]
        state.initial_speed = unpacked[21]
        state.projectile_allowance_17mm = unpacked[22]
        state.projectile_allowance_42mm = unpacked[23]
        state.rfid_status = unpacked[24]

        return state


def pack_ctrl_frame(ctrl_data: RobotCtrlData) -> bytes:
    """Pack a control frame for transmission to the C-board (NUC -> C-board).

    Args:
        ctrl_data: Populated RobotCtrlData instance.

    Returns:
        Complete frame bytes including header and CRC16.
    """
    data = ctrl_data.pack()
    frame = struct.pack('<BHB', SOF_TX, len(data), CMD_ID_CTRL) + data
    return append_crc16(frame)


def unpack_state_frame(raw_bytes: bytes) -> tuple:
    """Parse a state feedback frame received from the C-board.

    Args:
        raw_bytes: Raw bytes starting at the SOF byte.

    Returns:
        (success, state_data) tuple. success is False if the frame is
        incomplete, CRC fails, or cmd_id does not match.
    """
    if len(raw_bytes) < 6:
        return False, None

    if raw_bytes[0] != SOF_RX:
        return False, None

    data_len = struct.unpack('<H', raw_bytes[1:3])[0]
    cmd_id = raw_bytes[3]
    frame_len = 4 + data_len + 2

    if len(raw_bytes) < frame_len:
        return False, None

    if not verify_crc16(raw_bytes[:frame_len]):
        return False, None

    if cmd_id != CMD_ID_STATE:
        return False, None

    state = RobotStateData.unpack(raw_bytes[4:4 + data_len])
    return True, state
