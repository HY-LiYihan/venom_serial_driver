# venom_serial_driver

Serial driver for DJI C-board communication in ROS 2.

## Features

- Serial communication with DJI C-board
- Custom protocol implementation with CRC16 validation
- ROS 2 integration with custom messages (RobotStatus, GameStatus)
- Python-based implementation using pyserial

## Dependencies

- ROS 2 Humble
- Python 3
- pyserial

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/HY-LiYihan/venom_serial_driver.git
cd ~/ros2_ws
colcon build --packages-select venom_serial_driver
```

## Usage

```bash
ros2 launch venom_serial_driver serial_driver.launch.py
```

## License

MIT
