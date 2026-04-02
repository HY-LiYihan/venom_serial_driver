import logging
import threading
from typing import Optional

import serial


LOGGER = logging.getLogger(__name__)


class SerialInterface:
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    @property
    def serial(self) -> Optional[serial.Serial]:
        """Backward-compatible alias for the underlying pyserial object."""
        return self._ser

    @serial.setter
    def serial(self, value: Optional[serial.Serial]) -> None:
        self._ser = value

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            return True
        except Exception as e:
            LOGGER.error("Failed to connect serial port %s: %s", self.port, e)
            return False

    def disconnect(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_bytes(self, size: int = 1) -> bytes:
        if not self.is_connected():
            return b''
        try:
            with self.lock:
                return self._ser.read(size)  # type: ignore[union-attr]
        except Exception as e:
            LOGGER.warning("Serial read failed on %s: %s", self.port, e)
            return b''

    def write_bytes(self, data: bytes) -> int:
        if not self.is_connected():
            return 0
        try:
            with self.lock:
                return self._ser.write(data)  # type: ignore[union-attr]
        except Exception as e:
            LOGGER.warning("Serial write failed on %s: %s", self.port, e)
            return 0

    def is_connected(self) -> bool:
        return bool(self._ser and self._ser.is_open)
