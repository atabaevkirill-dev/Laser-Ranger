"""
Serial (UART TTL 3.3V) transport for the laser rangefinder protocol.
Baud rate: 115200 (default) / 57600 / 9600
"""

import time
import logging
import serial
from protocol_base import ProtocolHandlerBase


class ProtocolHandler(ProtocolHandlerBase):
    """Serial-port transport."""
    
    # Supported baud rates
    SUPPORTED_BAUD_RATES = [9600, 57600, 115200]
    
    # Default timeout (seconds)
    DEFAULT_TIMEOUT = 1.0
    
    MIN_BAUD_RATE = 1200
    MAX_BAUD_RATE = 230400

    def __init__(self, serial_port: serial.Serial, baud_rate: int = 115200, timeout: float = DEFAULT_TIMEOUT):
        """
        Initialize serial port handler.
        
        Args:
            serial_port: Pre-configured serial port object (may be already open)
            baud_rate: Baud rate
            timeout: Timeout in seconds
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If port fails to open
        """
        # 验证参数
        if baud_rate < self.MIN_BAUD_RATE or baud_rate > self.MAX_BAUD_RATE:
            raise ValueError(f"Baud rate must be between {self.MIN_BAUD_RATE} and {self.MAX_BAUD_RATE}")
            
        if timeout <= 0:
            raise ValueError("Timeout must be positive")
            
        self._logger = logging.getLogger(__name__)
        self._port = serial_port
        
        try:
            # Configure serial port parameters (port may already be open if passed pre-configured)
            self._port.baudrate = baud_rate
            self._port.timeout = timeout
            if not self._port.is_open:
                self._port.open()
            
            # Verify port is open
            if not self._port.is_open:
                raise RuntimeError("Failed to open serial port")
                
            self._logger.info(f"Serial port opened: {self._port.port}, baud: {baud_rate}, timeout: {timeout}s")
            
        except serial.SerialException as e:
            self._logger.error(f"Serial port init failed: {str(e)}")
            raise RuntimeError(f"Serial port init failed: {str(e)}") from e

    def _send_raw(self, data: bytes) -> None:
        """
        Send raw bytes over serial port.
        
        Args:
            data: Data to send
            
        Raises:
            RuntimeError: If send fails
        """
        try:
            self._logger.debug(f"Sending: {data.hex(' ')}")
            bytes_written = self._port.write(data)
            self._port.flush()
            
            if bytes_written != len(data):
                self._logger.warning(f"Partial send: {bytes_written}/{len(data)} bytes")
                
        except serial.SerialException as e:
            self._logger.error(f"Serial write failed: {str(e)}")
            raise RuntimeError(f"Serial write failed: {str(e)}") from e

    def _recv_bytes(self, n: int, timeout: float = None) -> bytes:
        """
        Read exactly n bytes from serial port.
        
        Args:
            n: Number of bytes to read
            timeout: Timeout in seconds
            
        Returns:
            Bytes read from port
            
        Raises:
            TimeoutError: If read times out
            RuntimeError: If serial error occurs
        """
        try:
            if timeout is not None and timeout > 0:
                self._port.timeout = timeout
            else:
                timeout = self._port.timeout
                
            deadline = time.monotonic() + timeout
            buf = bytearray()
            
            while len(buf) < n and time.monotonic() < deadline:
                waiting = self._port.in_waiting
                if waiting > 0:
                    chunk = self._port.read(min(n - len(buf), waiting))
                    buf.extend(chunk)
                    self._logger.debug(f"Received: {chunk.hex(' ')}")
                    
                if not buf:
                    time.sleep(0.005)
                    
            if not buf:
                raise TimeoutError("Serial read timeout")
                
            return bytes(buf)
            
        except serial.SerialException as e:
            self._logger.error(f"Serial read failed: {str(e)}")
            raise RuntimeError(f"Serial read failed: {str(e)}") from e

    def close(self):
        """Close the serial port."""
        if hasattr(self, '_port') and self._port.is_open:
            self._logger.info(f"Closing serial port: {self._port.port}")
            self._port.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False