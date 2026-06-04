"""
TCP/IP transport for the laser rangefinder protocol.
Wraps a connected socket.socket and delegates all protocol logic to
ProtocolHandlerBase – no code duplication.
"""

import socket
import logging
import time
from protocol_base import ProtocolHandlerBase

logger = logging.getLogger(__name__)


class TcpProtocolHandler(ProtocolHandlerBase):
    """TCP socket transport."""

    def __init__(self, sock: socket.socket):
        self._sock = sock
        # Enable TCP keepalive for reliable long-running connections
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, 'TCP_KEEPIDLE'):
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

    def _send_raw(self, data: bytes) -> None:
        try:
            self._sock.sendall(data)
        except OSError as e:
            logger.error(f"TCP send failed: {e}")
            raise RuntimeError(f"TCP send failed: {e}") from e

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
            except OSError as e:
                # Handles ConnectionResetError, BrokenPipeError, etc.
                logger.error(f"TCP recv error: {e}")
                raise OSError(f"TCP connection error: {e}") from e
            if not chunk:
                raise OSError("TCP connection closed by remote")
            buf.extend(chunk)
        return bytes(buf)

    def close(self):
        """Close the TCP socket."""
        if hasattr(self, '_sock') and self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None