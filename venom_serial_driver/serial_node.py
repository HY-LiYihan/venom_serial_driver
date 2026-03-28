import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from auto_aim_interfaces.msg import AutoAimCmd
from venom_serial_driver.msg import GameStatus, RobotStatus

from . import serial_protocol
from .serial_interface import SerialInterface


class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('loop_rate', 50)
        self.declare_parameter('auto_aim_topic', '/auto_aim')
        self.declare_parameter('vision_timeout', 0.2)
        self.declare_parameter('default_frame_x', 0)
        self.declare_parameter('default_frame_y', 0)

        port = self.get_parameter('port_name').value
        baudrate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        rate = self.get_parameter('loop_rate').value
        self.auto_aim_topic = self.get_parameter('auto_aim_topic').value
        self.vision_timeout = self.get_parameter('vision_timeout').value
        self.default_frame_x = self.get_parameter('default_frame_x').value
        self.default_frame_y = self.get_parameter('default_frame_y').value

        self.get_logger().info(f'Initializing serial: {port} @ {baudrate}')

        self.serial = SerialInterface(port, baudrate, timeout)
        if not self.serial.connect():
            self.get_logger().error('Failed to connect to serial port')
            return

        self.get_logger().info('Serial port connected')

        self.rx_buffer = bytearray()
        self.last_valid_time = time.time()

        self.latest_cmd_vel = Twist()
        self.latest_auto_aim = None
        self.latest_auto_aim_time = None

        self.robot_status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.game_status_pub = self.create_publisher(GameStatus, '/game_status', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.auto_aim_sub = self.create_subscription(
            AutoAimCmd, self.auto_aim_topic, self.auto_aim_callback, 10)

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        if not self.serial.is_connected():
            return

        data = self.serial.read_bytes(128)
        if data:
            self.rx_buffer.extend(data)

        while len(self.rx_buffer) >= 6:
            if self.rx_buffer[0] != serial_protocol.SOF_RX:
                self.rx_buffer.pop(0)
                continue

            success, state = serial_protocol.unpack_state_frame(bytes(self.rx_buffer))
            if success and state:
                robot_status = RobotStatus()
                robot_status.velocity.linear.x = float(state.linear_x)
                robot_status.velocity.linear.y = float(state.linear_y)
                robot_status.velocity.linear.z = float(state.linear_z)
                robot_status.velocity.angular.x = float(state.gyro_wz)
                robot_status.velocity.angular.y = float(state.angular_y)
                robot_status.velocity.angular.z = float(state.angular_z)
                robot_status.angular_speed.angular.y = float(state.angular_y_speed)
                robot_status.angular_speed.angular.z = float(state.angular_z_speed)
                self.robot_status_pub.publish(robot_status)

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

                self.last_valid_time = time.time()
                data_len = int.from_bytes(self.rx_buffer[1:3], 'little')
                frame_len = 4 + data_len + 2
                self.rx_buffer = self.rx_buffer[frame_len:]
            else:
                self.rx_buffer.pop(0)

        self.send_cached_control()

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def auto_aim_callback(self, msg):
        self.latest_auto_aim = msg
        self.latest_auto_aim_time = time.time()

    def is_recent(self, timestamp):
        return timestamp is not None and (time.time() - timestamp) < self.vision_timeout

    def send_cached_control(self):
        try:
            ctrl = serial_protocol.RobotCtrlData()
            ctrl.lx = float(self.latest_cmd_vel.linear.x)
            ctrl.ly = float(self.latest_cmd_vel.linear.y)
            ctrl.lz = float(self.latest_cmd_vel.linear.z)
            ctrl.ax = float(self.latest_cmd_vel.angular.x)

            has_auto_aim = self.is_recent(self.latest_auto_aim_time) and self.latest_auto_aim is not None

            flags = 0
            if has_auto_aim and self.latest_auto_aim.detected:
                flags |= 0x01
            if has_auto_aim and self.latest_auto_aim.tracking:
                flags |= 0x02
            if has_auto_aim and self.latest_auto_aim.fire:
                flags |= 0x04
            ctrl.flags = flags

            if has_auto_aim:
                ctrl.ay = float(self.latest_auto_aim.pitch)
                ctrl.az = float(self.latest_auto_aim.yaw)
                ctrl.dist = float(self.latest_auto_aim.distance)
                ctrl.frame_x = int(self.latest_auto_aim.proj_x)
                ctrl.frame_y = int(self.latest_auto_aim.proj_y)
            else:
                ctrl.ay = 0.0
                ctrl.az = 0.0
                ctrl.dist = 0.0
                ctrl.frame_x = int(self.default_frame_x)
                ctrl.frame_y = int(self.default_frame_y)

            frame = serial_protocol.pack_ctrl_frame(ctrl)
            self.serial.write_bytes(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to send ctrl: {e}')

    def is_online(self):
        return (time.time() - self.last_valid_time) < 0.1

    def destroy_node(self):
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
