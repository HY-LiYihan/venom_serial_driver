"""Launch file for the venom teleop keyboard control node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    topic = LaunchConfiguration('topic')

    declare_linear_speed = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.3',
        description='Linear translation speed (m/s)'
    )
    declare_angular_speed = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Chassis motion angular velocity (non-spin, rad/s)'
    )
    declare_topic = DeclareLaunchArgument(
        'topic',
        default_value='/cmd_vel',
        description='Topic to publish Twist commands on'
    )

    teleop_node = Node(
        package='venom_serial_driver',
        executable='teleop_node',
        name='venom_teleop',
        output='screen',
        parameters=[{
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'topic': topic,
        }],
        prefix='xterm -e',
    )

    return LaunchDescription([
        declare_linear_speed,
        declare_angular_speed,
        declare_topic,
        teleop_node,
    ])
