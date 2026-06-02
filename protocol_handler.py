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
    
    # 支持的波特率列表
    SUPPORTED_BAUD_RATES = [9600, 57600, 115200]
    
    # 默认超时时间（秒）
    DEFAULT_TIMEOUT = 1.0
    
    # 最小波特率
    MIN_BAUD_RATE = 1200
    
    # 最大波特率
    MAX_BAUD_RATE = 230400

    def __init__(self, serial_port: serial.Serial, baud_rate: int = 115200, timeout: float = DEFAULT_TIMEOUT):
        """
        初始化串口处理器
        
        Args:
            serial_port: 已配置的串口对象
            baud_rate: 波特率
            timeout: 超时时间（秒）
            
        Raises:
            ValueError: 如果参数无效
            RuntimeError: 如果端口打开失败
        """
        # 验证参数
        if baud_rate < self.MIN_BAUD_RATE or baud_rate > self.MAX_BAUD_RATE:
            raise ValueError(f"波特率必须在{self.MIN_BAUD_RATE}-{self.MAX_BAUD_RATE}之间")
            
        if timeout <= 0:
            raise ValueError("超时时间必须大于0")
            
        self._logger = logging.getLogger(__name__)
        self._port = serial_port
        
        try:
            # 配置并打开串口
            self._port.baudrate = baud_rate
            self._port.timeout = timeout
            self._port.open()
            
            # 验证端口是否成功打开
            if not self._port.is_open:
                raise RuntimeError("无法打开串口")
                
            self._logger.info(f"串口已打开: {self._port.port}, 波特率: {baud_rate}, 超时: {timeout}s")
            
        except serial.SerialException as e:
            self._logger.error(f"串口初始化失败: {str(e)}")
            raise RuntimeError(f"串口初始化失败: {str(e)}") from e

    def _send_raw(self, data: bytes) -> None:
        """
        发送原始字节数据
        
        Args:
            data: 要发送的数据
            
        Raises:
            RuntimeError: 如果发送失败
        """
        try:
            self._logger.debug(f"发送数据: {data.hex(' ')}")
            bytes_written = self._port.write(data)
            self._port.flush()  # 确保数据发送
            
            if bytes_written != len(data):
                self._logger.warning(f"未完全发送数据: {bytes_written}/{len(data)} 字节")
                
        except serial.SerialException as e:
            self._logger.error(f"串口写入失败: {str(e)}")
            raise RuntimeError(f"串口写入失败: {str(e)}") from e

    def _recv_bytes(self, n: int, timeout: float = None) -> bytes:
        """
        读取指定数量的字节
        
        Args:
            n: 要读取的字节数
            timeout: 超时时间（秒）
            
        Returns:
            读取到的字节数据
            
        Raises:
            TimeoutError: 如果读取超时
            RuntimeError: 如果发生串口错误
        """
        try:
            # 设置超时时间
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
                    self._logger.debug(f"接收数据: {chunk.hex(' ')}")
                    
                if not buf:  # 如果缓冲区仍然为空，等待一下
                    time.sleep(0.005)
                    
            if not buf:
                error_msg = "串口读取超时"
                self._logger.warning(error_msg)
                raise TimeoutError(error_msg)
                
            return bytes(buf)
            
        except serial.SerialException as e:
            self._logger.error(f"串口读取失败: {str(e)}")
            raise RuntimeError(f"串口读取失败: {str(e)}") from e
            
    def close(self):
        """关闭串口"""
        if hasattr(self, '_port') and self._port.is_open:
            self._logger.info(f"关闭串口: {self._port.port}")
            self._port.close()

    def __enter__(self):
        """上下文管理器进入"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.close()
        return False  # 不抑制异常