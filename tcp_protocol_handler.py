"""
TCP/IP transport for the laser rangefinder protocol.
Wraps a connected socket.socket and delegates all protocol logic to
ProtocolHandlerBase – no code duplication.
"""

import socket
import time
from protocol_base import ProtocolHandlerBase


class TcpProtocolHandler(ProtocolHandlerBase):
    """TCP socket transport."""

    def __init__(self, sock: socket.socket):
        self._sock = sock
        # Устанавливаем параметры сокета для более надежного соединения
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, 'TCP_KEEPIDLE'):
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)  # Начать проверки после 60 сек
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)  # Интервал между проверками
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)    # Количество попыток

    def _send_raw(self, data: bytes) -> None:
        self._sock.sendall(data)   # sendall is safer than send for TCP

    def _recv_bytes(self, n: int, timeout: float) -> bytes:
        """
        Read exactly n bytes from the socket within timeout seconds.
        Raises TimeoutError on timeout, OSError on connection error.
        """
        self._sock.settimeout(timeout)
        buf = bytearray()
        deadline = time.monotonic() + timeout
        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("TCP read timeout")
            try:
                self._sock.settimeout(remaining)
                chunk = self._sock.recv(n - len(buf))
            except socket.timeout:
                raise TimeoutError("TCP read timeout")
            if not chunk:
                raise OSError("TCP connection closed by remote")
            buf.extend(chunk)
        return bytes(buf)