#!/usr/bin/env python3
"""ROS 2 serial driver node for DJI C-board communication.

Bridges ROS 2 topics to the proprietary binary protocol used by the DJI C-board.
Subscribes to /cmd_vel (chassis motion) and /auto_aim (gimbal control + aim state),
caches the latest values, and sends a control frame whenever either input updates.
Optionally sends keepalive frames at a configured minimum frequency.
Publishes /robot_status and /game_status from incoming C-board state frames.
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from auto_aim_interfaces.msg import AutoAimCmd

if __package__:
    from venom_serial_driver.msg import RobotStatus, GameStatus
    from venom_serial_driver.serial_interface import SerialInterface
    from venom_serial_driver import serial_protocol
else:
    script_dir = os.path.dirname(__file__)
    install_site = os.path.normpath(os.path.join(script_dir, '..', 'python3.10', 'site-packages'))
    package_dir = os.path.join(install_site, 'venom_serial_driver')
    for p in (install_site, package_dir):
        if p not in sys.path:
            sys.path.insert(0, p)
    from venom_serial_driver.msg import RobotStatus, GameStatus
    from serial_interface import SerialInterface
    import serial_protocol


def _hexdump(data: bytes) -> str:
    """Format raw bytes as uppercase hex pairs."""
    return ' '.join(f'{byte:02X}' for byte in data)


class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Declare parameters
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('loop_rate', 50)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('auto_aim_topic', '/auto_aim')
        self.declare_parameter('venom_cmd_topic', '/venom_cmd')
        self.declare_parameter('robot_status_topic', '/robot_status')
        self.declare_parameter('game_status_topic', '/game_status')
        self.declare_parameter('vision_timeout', 0.2)
        self.declare_parameter('default_frame_x', 0)
        self.declare_parameter('default_frame_y', 0)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('heartbeat_rate', 0.0)

        # Retrieve parameters
        port = self.get_parameter('port_name').value
        baudrate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        rate = self.get_parameter('loop_rate').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        auto_aim_topic = self.get_parameter('auto_aim_topic').value
        venom_cmd_topic = self.get_parameter('venom_cmd_topic').value
        robot_status_topic = self.get_parameter('robot_status_topic').value
        game_status_topic = self.get_parameter('game_status_topic').value
        self._vision_timeout = self.get_parameter('vision_timeout').value
        self._default_frame_x = self.get_parameter('default_frame_x').value
        self._default_frame_y = self.get_parameter('default_frame_y').value
        self._pitch_sign = float(self.get_parameter('pitch_sign').value)
        self._yaw_sign = float(self.get_parameter('yaw_sign').value)
        self._heartbeat_rate_hz = float(self.get_parameter('heartbeat_rate').value)
        if self._heartbeat_rate_hz < 0.0:
            self.get_logger().warn(
                f'heartbeat_rate={self._heartbeat_rate_hz:.4f} is invalid; '
                'using 0.0 (disabled).'
            )
            self._heartbeat_rate_hz = 0.0
        self._heartbeat_period = (
            1.0 / self._heartbeat_rate_hz if self._heartbeat_rate_hz > 0.0 else 0.0
        )

        self.get_logger().info(f'Initializing serial: {port} @ {baudrate}')

        # Initialize serial interface
        self.serial = SerialInterface(port, baudrate, timeout)
        if not self.serial.connect():
            self.get_logger().error('[ERROR] Failed to connect to serial port')
            return

        self.get_logger().info('Serial port connected')

        # Receive buffer
        self.rx_buffer = bytearray()
        self.last_valid_time = time.time()

        # ---------------------------------------------------------------------------
        # Latest values from subscriptions (timer-driven merge)
        # ---------------------------------------------------------------------------
        self._latest_cmd_vel = Twist()
        self._latest_auto_aim = AutoAimCmd()
        self._latest_auto_aim_time: float | None = None
        self._tx_debug_counter = 0
        self._last_tx_debug_time: float | None = None
        self._last_ctrl_tx_time: float | None = None

        # ---------------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------------
        self.robot_status_pub = self.create_publisher(RobotStatus, robot_status_topic, 10)
        self.game_status_pub = self.create_publisher(GameStatus, game_status_topic, 10)
        self.venom_cmd_pub = self.create_publisher(String, venom_cmd_topic, 10)

        # ---------------------------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------------------------
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.auto_aim_sub = self.create_subscription(
            AutoAimCmd, auto_aim_topic, self._auto_aim_callback, 10)

        # Poll serial RX at a fixed rate; TX is event-driven with optional heartbeat.
        self.rx_timer = self.create_timer(1.0 / rate, self._poll_serial_rx)
        if self._heartbeat_period > 0.0:
            self.heartbeat_timer = self.create_timer(
                self._heartbeat_period, self._heartbeat_timer_callback
            )
            self.get_logger().info(
                f'Heartbeat enabled: {self._heartbeat_rate_hz:.3f} Hz '
                f'(min interval {self._heartbeat_period:.3f}s)'
            )
        else:
            self.heartbeat_timer = None
            self.get_logger().info('Heartbeat disabled (heartbeat_rate=0)')

    # ---------------------------------------------------------------------------
    # Subscription callbacks (cache only)
    # ---------------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Cache the latest chassis velocity command and trigger a TX frame."""
        if (
            abs(float(msg.linear.z)) > 1e-6
            or abs(float(msg.angular.x)) > 1e-6
            or abs(float(msg.angular.y)) > 1e-6
        ):
            self.get_logger().warn(
                'Received /cmd_vel with unsupported fields: '
                f'linear.z={msg.linear.z:.4f}, '
                f'angular.x={msg.angular.x:.4f}, '
                f'angular.y={msg.angular.y:.4f}. '
                'Expected usage: linear.x/y + angular.z only.'
            )
        self._latest_cmd_vel = msg
        self._send_cached_ctrl_frame('cmd_vel')

    def _auto_aim_callback(self, msg: AutoAimCmd) -> None:
        """Cache the latest auto-aim command, timestamp it, and trigger a TX frame."""
        self._latest_auto_aim = msg
        self._latest_auto_aim_time = time.time()
        self._send_cached_ctrl_frame('auto_aim')

    def _is_auto_aim_recent(self) -> bool:
        """Return True if an auto-aim message arrived within vision_timeout seconds."""
        return (
            self._latest_auto_aim_time is not None
            and (time.time() - self._latest_auto_aim_time) < self._vision_timeout
        )

    # ---------------------------------------------------------------------------
    # Timer callback: read serial RX only
    # ---------------------------------------------------------------------------

    def _poll_serial_rx(self) -> None:
        """Read incoming serial data and publish decoded status topics."""
        if not self.serial.is_connected():
            return

        # Read data into buffer
        data = self.serial.read_bytes(128)
        if data:
            self.rx_buffer.extend(data)

        # Sliding-window frame parsing
        while len(self.rx_buffer) >= 6:
            if self.rx_buffer[0] != serial_protocol.SOF_RX:
                self.rx_buffer.pop(0)
                continue

            success, state = serial_protocol.unpack_state_frame(bytes(self.rx_buffer))

            if success and state:
                self._publish_robot_status(state)
                self._publish_game_status(state)
                self.last_valid_time = time.time()

                data_len = int.from_bytes(self.rx_buffer[1:3], 'little')
                frame_len = 4 + data_len + 2
                self.rx_buffer = self.rx_buffer[frame_len:]
            else:
                self.rx_buffer.pop(0)

    def _heartbeat_timer_callback(self) -> None:
        """Maintain a minimum TX frequency when heartbeat_rate is enabled."""
        if self._heartbeat_period <= 0.0 or not self.serial.is_connected():
            return

        now = time.time()
        if (
            self._last_ctrl_tx_time is not None
            and (now - self._last_ctrl_tx_time) < self._heartbeat_period
        ):
            return

        self._send_cached_ctrl_frame('heartbeat')

    # ---------------------------------------------------------------------------
    # Control frame: merge cached /cmd_vel and /auto_aim, send to C-board
    # ---------------------------------------------------------------------------

    def _send_cached_ctrl_frame(self, trigger_source: str) -> None:
        """Send a control frame composed from the latest cached control inputs.

        Field mapping (matches Vision_navigation_ctrl_payload_t in firmware):
            chassis_vx        <- /cmd_vel linear.x      (chassis velocity, m/s)
            chassis_vy        <- /cmd_vel linear.y      (chassis velocity, m/s)
            chassis_motion_wz <- /cmd_vel angular.z     (non-spin motion angular velocity, rad/s)
            chassis_world_wz  <- fixed 0.0              (world-frame spin, not used for motion control)
            angular_y         <- /auto_aim pitch        (gimbal pitch angle, rad)
            angular_z         <- /auto_aim yaw          (gimbal yaw angle, rad)
            flags       <- /auto_aim detected/tracking/fire (bit0/1/2)
            dist        <- /auto_aim distance
            frame_x     <- /auto_aim proj_x
            frame_y     <- /auto_aim proj_y
        """
        if not self.serial.is_connected():
            return

        cmd = self._latest_cmd_vel
        ctrl = serial_protocol.RobotCtrlData()

        # Chassis motion from /cmd_vel.
        ctrl.chassis_vx = float(cmd.linear.x)
        ctrl.chassis_vy = float(cmd.linear.y)
        # chassis_motion_wz carries chassis motion angular velocity (non-spin).
        ctrl.chassis_motion_wz = float(cmd.angular.z)
        # chassis_world_wz is not used in current motion control path.
        ctrl.chassis_world_wz = 0.0

        # Gimbal control and aim state from /auto_aim
        aim = self._latest_auto_aim if self._is_auto_aim_recent() else None
        if aim is not None:
            ctrl.angular_y = self._pitch_sign * float(aim.pitch)
            ctrl.angular_z = self._yaw_sign * float(aim.yaw)
            ctrl.distance = float(aim.distance)
            ctrl.frame_x = int(aim.proj_x)
            ctrl.frame_y = int(aim.proj_y)
            flags = 0
            if aim.detected:
                flags |= 0x01
            if aim.tracking:
                flags |= 0x02
            if aim.fire:
                flags |= 0x04
            ctrl.flags = flags
        else:
            ctrl.angular_y = 0.0
            ctrl.angular_z = 0.0
            ctrl.distance = 0.0
            ctrl.frame_x = self._default_frame_x
            ctrl.frame_y = self._default_frame_y
            ctrl.status_flags = 0

        try:
            frame = serial_protocol.pack_ctrl_frame(ctrl)
            self.serial.write_bytes(frame)
            tx_now = time.time()
            self._last_ctrl_tx_time = tx_now
            debug_msg = String()
            debug_msg.data = (
                '[TX:%s] flags=%d vx=%.4f vy=%.4f motion_wz=%.4f world_wz=%.4f '
                'pitch=%.4f yaw=%.4f dist=%.4f frame=(%d,%d) raw=%s' % (
                    trigger_source,
                    ctrl.status_flags,
                    ctrl.chassis_vx,
                    ctrl.chassis_vy,
                    ctrl.chassis_motion_wz,
                    ctrl.chassis_world_wz,
                    ctrl.angular_y,
                    ctrl.angular_z,
                    ctrl.distance,
                    ctrl.frame_x,
                    ctrl.frame_y,
                    _hexdump(frame),
                )
            )
            self.venom_cmd_pub.publish(debug_msg)
            self._tx_debug_counter += 1
            tx_hz = 0.0
            if self._last_tx_debug_time is not None:
                dt = tx_now - self._last_tx_debug_time
                if dt > 1e-6:
                    tx_hz = 1.0 / dt
            self._last_tx_debug_time = tx_now
            # Keep the detailed TX log disabled to avoid continuous console spam.
            # self.get_logger().info(
            #     '[TX:%s] t=%.3f hz=%.1f flags=%d lx=%.4f ly=%.4f lz=%.4f wz=%.4f '
            #     'pitch=%.4f yaw=%.4f dist=%.4f frame=(%d,%d) raw=%s' % (
            #         trigger_source,
            #         now,
            #         tx_hz,
            #         ctrl.flags,
            #         ctrl.lx,
            #         ctrl.ly,
            #         ctrl.lz,
            #         ctrl.chassis_wz,
            #         ctrl.ay,
            #         ctrl.az,
            #         ctrl.dist,
            #         ctrl.frame_x,
            #         ctrl.frame_y,
            #         _hexdump(frame),
            #     )
            # )
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to send ctrl frame: {e}')

    # ---------------------------------------------------------------------------
    # Status publishers
    # ---------------------------------------------------------------------------

    def _publish_robot_status(self, state) -> None:
        """Publish robot velocity and gimbal angle status."""
        robot_status = RobotStatus()
        robot_status.velocity.linear.x = float(state.chassis_vx)
        robot_status.velocity.linear.y = float(state.chassis_vy)
        robot_status.velocity.linear.z = float(state.chassis_motion_wz)
        robot_status.velocity.angular.x = float(state.chassis_world_wz)
        robot_status.velocity.angular.y = float(state.angular_y)
        robot_status.velocity.angular.z = float(state.angular_z)
        robot_status.angular_speed.angular.y = float(state.angular_y_speed)
        robot_status.angular_speed.angular.z = float(state.angular_z_speed)
        self.robot_status_pub.publish(robot_status)

    def _publish_game_status(self, state) -> None:
        """Publish RoboMaster game status and shooter heat data."""
        game_status = GameStatus()
        game_status.timestamp_us = state.timestamp_us
        game_status.game_progress = state.game_progress
        game_status.stage_remain_time = state.stage_remain_time
        game_status.center_outpost_occupancy = state.center_outpost_occupancy
        game_status.hp_percentage = (
            float(state.current_HP) / float(state.maximum_HP)
            if state.maximum_HP > 0 else 0.0
        )
        game_status.shooter_barrel_heat_limit = state.shooter_barrel_heat_limit
        game_status.power_management = state.power_management
        game_status.shooter_17mm_barrel_heat = state.shooter_17mm_barrel_heat
        game_status.shooter_42mm_barrel_heat = state.shooter_42mm_barrel_heat
        game_status.armor_id = state.armor_id
        game_status.hp_deduction_reason = state.HP_deduction_reason
        game_status.launching_frequency = float(state.launching_frequency)
        game_status.initial_speed = float(state.initial_speed)
        game_status.projectile_allowance_17mm = state.projectile_allowance_17mm
        game_status.projectile_allowance_42mm = state.projectile_allowance_42mm
        game_status.rfid_status = state.rfid_status
        game_status.distance = float(state.distance)
        self.game_status_pub.publish(game_status)

    # ---------------------------------------------------------------------------
    # Lifecycle
    # ---------------------------------------------------------------------------

    def is_online(self) -> bool:
        """Return True if a valid frame was received within the last 100 ms."""
        return (time.time() - self.last_valid_time) < 0.1

    def destroy_node(self) -> None:
        """Disconnect serial port on node shutdown."""
        self.serial.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
