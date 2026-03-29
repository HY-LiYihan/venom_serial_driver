#!/usr/bin/env python3
"""Omnidirectional keyboard teleop node for the Venom robot chassis.

Publishes geometry_msgs/Twist to /venom_cmd_vel. Supports full holonomic
translation (linear.x/y) and chassis rotation (angular.z) for spin-top mode.
Designed for the DJI C-board serial driver which maps:
    linear.x/y -> chassis translation (m/s)
    angular.z  -> chassis_wz rotation (rad/s)

Key bindings:
    W / S      forward / backward  (+/- linear.x)
    A / D      strafe left / right (-/+ linear.y)
    Q / Z      spin CCW / CW       (+/- angular.z)
    E          stop rotation       (angular.z = 0)
    R / F      increase / decrease linear speed
    T / G      increase / decrease angular speed
    Ctrl-C     quit
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# ---------------------------------------------------------------------------
# Help text
# ---------------------------------------------------------------------------

HELP_MSG = """
Venom Omnidirectional Teleop
-----------------------------
Movement (linear):
  W          forward  (+linear.x)
  S          backward (-linear.x)
  A          strafe left  (-linear.y)
  D          strafe right (+linear.y)

Spin-top (angular.z):
  Q          spin CCW (+angular.z)
  Z          spin CW  (-angular.z)
  E          stop rotation

Speed adjustment:
  R / F      linear speed  x1.1 / x0.9
  T / G      angular speed x1.1 / x0.9

Any other key: stop translation (rotation unchanged)
Ctrl-C: quit
"""

# ---------------------------------------------------------------------------
# Key -> (dx, dy, dth) multipliers
# ---------------------------------------------------------------------------

MOVE_BINDINGS = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'z': (0, 0, -1),
    'e': (0, 0, 0),   # stop rotation sentinel — handled separately
}

SPEED_BINDINGS = {
    'r': (1.1, 1.0),
    'f': (0.9, 1.0),
    't': (1.0, 1.1),
    'g': (1.0, 0.9),
}

# ---------------------------------------------------------------------------
# Terminal helpers (copied from teleop_twist_keyboard pattern)
# ---------------------------------------------------------------------------


def get_key(settings):
    """Read a single keypress from stdin."""
    if sys.platform == 'win32':
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    """Save current terminal settings."""
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    """Restore terminal settings saved by save_terminal_settings."""
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    settings = save_terminal_settings()

    rclpy.init()
    node = rclpy.create_node('venom_teleop')

    linear_speed = node.declare_parameter('linear_speed', 0.3).value
    angular_speed = node.declare_parameter('angular_speed', 1.0).value
    topic = node.declare_parameter('topic', '/venom_cmd_vel').value

    pub = node.create_publisher(Twist, topic, 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    # Current angular.z state persists across translation key presses.
    current_angular = 0.0

    def publish_zero():
        msg = Twist()
        pub.publish(msg)

    try:
        print(HELP_MSG)
        print(f'linear speed: {linear_speed:.2f} m/s  |  angular speed: {angular_speed:.2f} rad/s')

        while True:
            key = get_key(settings)

            if key == '\x03':  # Ctrl-C
                break

            if key in SPEED_BINDINGS:
                lin_mul, ang_mul = SPEED_BINDINGS[key]
                linear_speed *= lin_mul
                angular_speed *= ang_mul
                print(f'linear speed: {linear_speed:.2f} m/s  |  angular speed: {angular_speed:.2f} rad/s')
                continue

            msg = Twist()

            if key in MOVE_BINDINGS:
                dx, dy, dth = MOVE_BINDINGS[key]
                msg.linear.x = dx * linear_speed
                msg.linear.y = dy * linear_speed

                if key == 'e':
                    # Stop rotation only; keep translation at zero (key released)
                    current_angular = 0.0
                elif dth != 0:
                    # Rotation key: update persistent angular state
                    current_angular = dth * angular_speed
                    # No translation while spinning unless combined
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
            else:
                # Unknown key: stop translation, keep rotation
                msg.linear.x = 0.0
                msg.linear.y = 0.0

            msg.angular.z = current_angular
            pub.publish(msg)

    except Exception as e:
        print(e)

    finally:
        publish_zero()
        rclpy.shutdown()
        spinner.join()
        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()
