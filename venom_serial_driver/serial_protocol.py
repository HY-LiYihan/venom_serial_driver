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
        status_flags      (uint8)
        chassis_vx        (float) - chassis velocity along +x (m/s)
        chassis_vy        (float) - chassis velocity along +y (m/s)
        chassis_motion_wz (float) - chassis motion angular velocity (non-spin, rad/s)
        chassis_world_wz  (float) - chassis rotation angular velocity in world frame (rad/s)
        angular_y         (float) - gimbal pitch angle command (rad, single-turn)
        angular_z         (float) - gimbal yaw angle command (rad, single-turn)
        distance          (float) - reserved
        frame_x           (uint16) - target pixel x
        frame_y           (uint16) - target pixel y
    """

    def __init__(self):
        self.status_flags = 0
        self.chassis_vx = 0.0
        self.chassis_vy = 0.0
        self.chassis_motion_wz = 0.0
        self.chassis_world_wz = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        self.distance = 0.0
        self.frame_x = 0
        self.frame_y = 0

    def pack(self) -> bytes:
        """Pack into little-endian byte stream. Format: <B7f2H."""
        return struct.pack('<B7f2H',
                          self.status_flags,
                          self.chassis_vx, self.chassis_vy, self.chassis_motion_wz,
                          self.chassis_world_wz, self.angular_y, self.angular_z,
                          self.distance,
                          self.frame_x, self.frame_y)

    # -----------------------------------------------------------------------
    # Backward-compatible aliases for older code/tests
    # -----------------------------------------------------------------------

    @property
    def flags(self):
        return self.status_flags

    @flags.setter
    def flags(self, value):
        self.status_flags = value

    @property
    def lx(self):
        return self.chassis_vx

    @lx.setter
    def lx(self, value):
        self.chassis_vx = value

    @property
    def ly(self):
        return self.chassis_vy

    @ly.setter
    def ly(self, value):
        self.chassis_vy = value

    @property
    def lz(self):
        return self.chassis_motion_wz

    @lz.setter
    def lz(self, value):
        self.chassis_motion_wz = value

    @property
    def chassis_wz(self):
        return self.chassis_world_wz

    @chassis_wz.setter
    def chassis_wz(self, value):
        self.chassis_world_wz = value

    @property
    def ay(self):
        return self.angular_y

    @ay.setter
    def ay(self, value):
        self.angular_y = value

    @property
    def az(self):
        return self.angular_z

    @az.setter
    def az(self, value):
        self.angular_z = value

    @property
    def dist(self):
        return self.distance

    @dist.setter
    def dist(self, value):
        self.distance = value


class RobotStateData:
    """State data received from C-board (Vision_navigation_feedback_payload_t).

    Packed format: <I9fBHB3HB2HB2f2HI (72 bytes total).
    """

    def __init__(self):
        self.timestamp_us = 0
        self.chassis_vx = 0.0
        self.chassis_vy = 0.0
        self.chassis_motion_wz = 0.0
        self.chassis_world_wz = 0.0
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
        state.chassis_vx = unpacked[1]
        state.chassis_vy = unpacked[2]
        state.chassis_motion_wz = unpacked[3]
        state.chassis_world_wz = unpacked[4]
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

    # -----------------------------------------------------------------------
    # Backward-compatible aliases for older code/tests
    # -----------------------------------------------------------------------

    @property
    def linear_x(self):
        return self.chassis_vx

    @linear_x.setter
    def linear_x(self, value):
        self.chassis_vx = value

    @property
    def linear_y(self):
        return self.chassis_vy

    @linear_y.setter
    def linear_y(self, value):
        self.chassis_vy = value

    @property
    def linear_z(self):
        return self.chassis_motion_wz

    @linear_z.setter
    def linear_z(self, value):
        self.chassis_motion_wz = value

    @property
    def gyro_wz(self):
        return self.chassis_world_wz

    @gyro_wz.setter
    def gyro_wz(self, value):
        self.chassis_world_wz = value


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
