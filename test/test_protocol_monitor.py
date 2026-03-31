#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from venom_serial_driver.serial_interface import SerialInterface
from venom_serial_driver import serial_protocol


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class ProtocolMonitor:
    def __init__(self, port: str, baudrate: int, timeout: float, show_raw: bool):
        self.serial = SerialInterface(port, baudrate, timeout)
        self.rx_buffer = bytearray()
        self.show_raw = show_raw
        self.frame_count = 0
        self.start_time = None

    def send_test_frame(self, yaw_rad: float, pitch_rad: float, detected: bool,
                        tracking: bool, fire: bool, distance: float,
                        frame_x: int, frame_y: int) -> None:
        ctrl = serial_protocol.RobotCtrlData()
        ctrl.flags = (
            (0x01 if detected else 0) |
            (0x02 if tracking else 0) |
            (0x04 if fire else 0)
        )
        ctrl.ay = pitch_rad
        ctrl.az = yaw_rad
        ctrl.dist = distance
        ctrl.frame_x = frame_x
        ctrl.frame_y = frame_y
        frame = serial_protocol.pack_ctrl_frame(ctrl)
        self.serial.write_bytes(frame)
        print("TX ctrl frame:")
        print(f"  flags={ctrl.flags:#04x} pitch={ctrl.ay:.4f}rad yaw={ctrl.az:.4f}rad "
              f"dist={ctrl.dist:.2f} frame=({ctrl.frame_x},{ctrl.frame_y})")
        print(f"  raw={hexdump(frame)}")

    def run(self) -> None:
        if not self.serial.connect():
            print("❌ 串口连接失败")
            return

        print(f"✓ 串口已连接: {self.serial.port} @ {self.serial.baudrate}")
        print("开始监听协议帧... (Ctrl+C 退出)\n")
        self.start_time = time.time()

        try:
            while True:
                data = self.serial.read_bytes(256)
                if data:
                    self.rx_buffer.extend(data)
                    self._parse_rx_frames()
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\n停止监听")
            self._print_stats()
        finally:
            self.serial.disconnect()

    def _parse_rx_frames(self) -> None:
        while len(self.rx_buffer) >= 6:
            if self.rx_buffer[0] != serial_protocol.SOF_RX:
                self.rx_buffer.pop(0)
                continue

            data_len = int.from_bytes(self.rx_buffer[1:3], "little")
            frame_len = 4 + data_len + 2
            if len(self.rx_buffer) < frame_len:
                return

            frame = bytes(self.rx_buffer[:frame_len])
            success, state = serial_protocol.unpack_state_frame(frame)
            if success and state:
                self.frame_count += 1
                elapsed = max(time.time() - self.start_time, 1e-6)
                freq = self.frame_count / elapsed
                print(f"[RX {self.frame_count:05d}] "
                      f"pitch={state.angular_y:8.4f}rad "
                      f"yaw={state.angular_z:8.4f}rad "
                      f"pitch_v={state.angular_y_speed:7.2f} "
                      f"yaw_v={state.angular_z_speed:7.2f} "
                      f"dist={state.distance:5.2f} "
                      f"v0={state.initial_speed:5.2f} "
                      f"HP={state.current_HP}/{state.maximum_HP} "
                      f"freq={freq:5.1f}Hz")
                if self.show_raw:
                    print(f"          raw={hexdump(frame)}")
                self.rx_buffer = self.rx_buffer[frame_len:]
            else:
                if self.show_raw:
                    print(f"[DROP] raw={hexdump(frame)}")
                self.rx_buffer.pop(0)

    def _print_stats(self) -> None:
        elapsed = max(time.time() - self.start_time, 1e-6)
        print(f"统计: {self.frame_count} 帧, 平均 {self.frame_count / elapsed:.2f} Hz")


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor vision serial protocol frames.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--raw", action="store_true", help="Print raw frame hex dump")
    parser.add_argument("--send-test", action="store_true",
                        help="Send one test control frame before listening")
    parser.add_argument("--yaw", type=float, default=0.0873, help="Test yaw in rad")
    parser.add_argument("--pitch", type=float, default=-0.0349, help="Test pitch in rad")
    parser.add_argument("--distance", type=float, default=2.5)
    parser.add_argument("--frame-x", type=int, default=640)
    parser.add_argument("--frame-y", type=int, default=360)
    parser.add_argument("--detected", action="store_true")
    parser.add_argument("--tracking", action="store_true")
    parser.add_argument("--fire", action="store_true")
    args = parser.parse_args()

    monitor = ProtocolMonitor(args.port, args.baud, args.timeout, args.raw)
    if not monitor.serial.connect():
        print("❌ 串口连接失败")
        return

    try:
        if args.send_test:
            monitor.send_test_frame(
                yaw_rad=args.yaw,
                pitch_rad=args.pitch,
                detected=args.detected,
                tracking=args.tracking,
                fire=args.fire,
                distance=args.distance,
                frame_x=args.frame_x,
                frame_y=args.frame_y,
            )
        print(f"✓ 串口已连接: {monitor.serial.port} @ {monitor.serial.baudrate}")
        print("开始监听协议帧... (Ctrl+C 退出)\n")
        monitor.start_time = time.time()
        while True:
            data = monitor.serial.read_bytes(256)
            if data:
                monitor.rx_buffer.extend(data)
                monitor._parse_rx_frames()
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n停止监听")
        monitor._print_stats()
    finally:
        monitor.serial.disconnect()


if __name__ == "__main__":
    main()
