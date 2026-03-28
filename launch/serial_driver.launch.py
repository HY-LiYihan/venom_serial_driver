"""Launch file for the venom_serial_driver node.

Starts the serial bridge between ROS2 and the DJI C-board over a serial port.
Aborts with a clear error message if the specified port device does not exist.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _check_port(context, *args, **kwargs):
    """Abort launch with a clear error if the serial port device does not exist."""
    port = LaunchConfiguration('port_name').perform(context)
    if not os.path.exists(port):
        sys.exit(
            f'[ERROR] Serial port {port!r} does not exist. '
            f'Check the device connection and the port_name argument.'
        )
    return []


def generate_launch_description():
    pkg_dir = get_package_share_directory('venom_serial_driver')
    config_file = os.path.join(pkg_dir, 'config', 'serial_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port name'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='921600',
            description='Baud rate'
        ),
        OpaqueFunction(function=_check_port),
        Node(
            package='venom_serial_driver',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'port_name': LaunchConfiguration('port_name'),
                    'baud_rate': LaunchConfiguration('baud_rate'),
                }
            ]
        )
    ])
