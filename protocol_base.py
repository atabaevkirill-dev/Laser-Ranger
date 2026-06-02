"""
Base protocol handler for the 3km eye-safe laser rangefinder module.
Implements the full communication protocol per the specification (v1.2, 2022.12).

Packet format:
  [0xEE][0x16][DataLen][0x03][CmdCode][Params...][CheckSum]
  DataLen = 1 (device) + 1 (cmd) + len(params)
  CheckSum = sum(device_code, cmd_code, params...) & 0xFF
"""

import time


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

FRAME_HEADER = (0xEE, 0x16)
DEVICE_CODE = 0x03

# Control commands (host → module)
CMD_SELF_CHECK              = 0x01
CMD_SINGLE_RANGING          = 0x02
CMD_SET_TARGET_MODE         = 0x03
CMD_CONTINUE_RANGING        = 0x04
CMD_STOP_RANGING            = 0x05
CMD_RANGING_ABNORMAL        = 0x06  # response only
CMD_LOW_POWER_WAKEUP        = 0x07  # response only

CMD_SET_BAUD_RATE           = 0xA0
CMD_SET_FREQUENCY           = 0xA1
CMD_SET_MIN_DISTANCE        = 0xA2
CMD_QUERY_MIN_DISTANCE      = 0xA3
CMD_SET_MAX_DISTANCE        = 0xA4
CMD_QUERY_MAX_DISTANCE      = 0xA5
CMD_QUERY_FPGA_VERSION      = 0xA6
CMD_QUERY_MCU_VERSION       = 0xA7
CMD_QUERY_HARDWARE_VERSION  = 0xA8
CMD_QUERY_SN_NUMBER         = 0xA9
CMD_QUERY_TOTAL_PULSES      = 0x90
CMD_QUERY_SESSION_PULSES    = 0x91

# Target modes
TARGET_FIRST = 0x01
TARGET_LAST  = 0x02
TARGET_MULTI = 0x03

FPGA_AUTHORS = {0x6C: "cliu", 0x5D: "dwu",  0xCC: "cycheng"}
MCU_AUTHORS  = {0x00: "jyang", 0xF1: "llfu", 0x01: "zqxiong"}


# ---------------------------------------------------------------------------
# Pure protocol helpers (no I/O)
# ---------------------------------------------------------------------------

def _checksum(data: list) -> int:
    return sum(data) & 0xFF


def build_packet(command_code: int, params: list = None) -> bytes:
    """Build a complete protocol packet."""
    if params is None:
        params = []
    data_len = 2 + len(params)          # device_code + cmd_code + params
    body = [DEVICE_CODE, command_code] + params
    return bytes(list(FRAME_HEADER) + [data_len] + body + [_checksum(body)])


def parse_packet(raw: list) -> dict | None:
    """
    Parse a complete raw packet (list of ints).
    Returns dict or None on error.
    """
    if len(raw) < 6:
        return None
    if raw[0] != 0xEE or raw[1] != 0x16:
        return None
    body        = raw[3:-1]          # device_code … params
    checksum    = raw[-1]
    if _checksum(body) != checksum:
        return None
    return {
        'device_code':  raw[3],
        'command_code': raw[4],
        'params':       raw[5:-1],
        'checksum':     checksum,
    }


def decode_ranging_response(params: list) -> dict:
    """
    Decode the 4-byte params of a single/continuous ranging response.
    Returns: distance (float, metres), status (int), status_description (str),
             is_multi_target (bool), target_number (int).
    """
    status, dist_h, dist_l, dist_dec = params
    distance = dist_h * 256 + dist_l + dist_dec * 0.1

    target_num = (status >> 4) & 0x0F
    target_type = status & 0x0F
    is_multi = target_num > 0

    _type_map = {
        0x0: "Single target",
        0x1: "Front target",
        0x2: "Rear target",
        0x3: "Front & rear targets",
        0x4: "Out of range",
    }
    type_desc = _type_map.get(target_type, f"Reserved (0x{target_type:X})")

    if is_multi:
        status_description = f"Target #{target_num}: {type_desc}"
    else:
        status_description = type_desc

    return {
        'distance':           distance,
        'status':             status,
        'status_description': status_description,
        'is_multi_target':    is_multi,
        'target_number':      target_num,
    }


def decode_version_params(params: list, authors: dict) -> dict:
    """Decode version response params (4 bytes)."""
    version_byte, date, month_year, author = params
    major = (version_byte >> 4) & 0x0F
    minor = version_byte & 0x0F
    month = (month_year >> 4) & 0x0F
    year  = 2020 + (month_year & 0x0F)
    return {
        'version': f"V{major}.{minor}",
        'date':    date,
        'month':   month,
        'year':    year,
        'author':  authors.get(author, f"Unknown(0x{author:02X})"),
    }


# ---------------------------------------------------------------------------
# Abstract base – subclasses implement _send_raw / _recv_bytes
# ---------------------------------------------------------------------------

class ProtocolHandlerBase:
    """
    Transport-agnostic protocol handler.
    Subclasses must implement:
        _send_raw(data: bytes) -> None
        _recv_bytes(n: int, timeout: float) -> bytes   (raises on timeout)
    """

    # expose constants as class attributes for backwards compat
    CMD_SELF_CHECK             = CMD_SELF_CHECK
    CMD_SINGLE_RANGING         = CMD_SINGLE_RANGING
    CMD_SET_TARGET_MODE        = CMD_SET_TARGET_MODE
    CMD_CONTINUE_RANGING       = CMD_CONTINUE_RANGING
    CMD_STOP_RANGING           = CMD_STOP_RANGING
    CMD_SET_BAUD_RATE          = CMD_SET_BAUD_RATE
    CMD_SET_FREQUENCY          = CMD_SET_FREQUENCY
    CMD_SET_MIN_DISTANCE       = CMD_SET_MIN_DISTANCE
    CMD_QUERY_MIN_DISTANCE     = CMD_QUERY_MIN_DISTANCE
    CMD_SET_MAX_DISTANCE       = CMD_SET_MAX_DISTANCE
    CMD_QUERY_MAX_DISTANCE     = CMD_QUERY_MAX_DISTANCE
    CMD_QUERY_FPGA_VERSION     = CMD_QUERY_FPGA_VERSION
    CMD_QUERY_MCU_VERSION      = CMD_QUERY_MCU_VERSION
    CMD_QUERY_HARDWARE_VERSION = CMD_QUERY_HARDWARE_VERSION
    CMD_QUERY_SN_NUMBER        = CMD_QUERY_SN_NUMBER
    CMD_QUERY_TOTAL_PULSES     = CMD_QUERY_TOTAL_PULSES
    CMD_QUERY_SESSION_PULSES   = CMD_QUERY_SESSION_PULSES
    TARGET_FIRST = TARGET_FIRST
    TARGET_LAST  = TARGET_LAST
    TARGET_MULTI = TARGET_MULTI

    # ------------------------------------------------------------------
    # Transport primitives – must be overridden
    # ------------------------------------------------------------------

    def _send_raw(self, data: bytes) -> None:
        raise NotImplementedError

    def _recv_bytes(self, n: int, timeout: float) -> bytes:
        """Read exactly n bytes. Raise TimeoutError if not available in time."""
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Internal I/O
    # ------------------------------------------------------------------

    def send_command(self, command_code: int, params: list = None) -> bool:
        try:
            self._send_raw(build_packet(command_code, params))
            return True
        except Exception as exc:
            print(f"[send] cmd=0x{command_code:02X} error: {exc}")
            return False

    def read_response(self, timeout: float = 2.0) -> list | None:
        """
        Read and return one complete packet as list[int], or None on failure.
        Scans for the 0xEE 0x16 header to recover from partial/garbage data.
        """
        deadline = time.monotonic() + timeout
        buf = bytearray()
        try:
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                # Read one byte at a time until we find the header
                b = self._recv_bytes(1, min(remaining, 0.1))
                buf.extend(b)
                # Need at least 2 bytes for header check
                if len(buf) < 2:
                    continue
                # Find 0xEE 0x16
                idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == 0xEE and buf[i + 1] == 0x16:
                        idx = i
                        break
                if idx == -1:
                    buf = buf[-1:]   # keep last byte (might start header)
                    continue
                buf = buf[idx:]     # trim before header
                if len(buf) < 3:
                    continue        # need length byte
                data_len = buf[2]
                total    = 3 + data_len + 1  # header(2) + len(1) + body + checksum
                while len(buf) < total and time.monotonic() < deadline:
                    remaining = deadline - time.monotonic()
                    need = total - len(buf)
                    chunk = self._recv_bytes(need, min(remaining, 0.5))
                    buf.extend(chunk)
                if len(buf) < total:
                    break
                packet = list(buf[:total])
                parsed = parse_packet(packet)
                if parsed is None:
                    # Bad checksum – discard header and try again
                    buf = buf[2:]
                    continue
                return packet
        except TimeoutError:
            pass
        except Exception as exc:
            print(f"[recv] error: {exc}")
        return None

    def _parse_response(self, raw: list) -> dict | None:
        """Public wrapper for parse_packet (backwards compatibility)."""
        return parse_packet(raw)

    def _decode_ranging_status(self, status: int) -> str:
        """Backwards-compatible single-value decode."""
        result = decode_ranging_response([status, 0, 0, 0])
        return result['status_description']

    def _decode_ranging_status_extended(self, status: int) -> tuple:
        """Returns (description, is_multi_target)."""
        result = decode_ranging_response([status, 0, 0, 0])
        return result['status_description'], result['is_multi_target']

    # ------------------------------------------------------------------
    # High-level commands
    # ------------------------------------------------------------------

    def self_check(self) -> dict | None:
        self.send_command(CMD_SELF_CHECK)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_SELF_CHECK or len(p['params']) != 4:
            return None
        s3, s2, s1, s0 = p['params']
        return {
            'echo_intensity':     s2,
            'fpga_ok':            bool(s1 & 0x01),
            'laser_output':       bool(s1 & 0x02),
            'main_wave':          bool(s1 & 0x04),
            'echo_detected':      bool(s1 & 0x08),
            'bias_switch':        bool(s1 & 0x10),
            'bias_output_ok':     bool(s1 & 0x20),
            'temperature_ok':     bool(s1 & 0x40),
            'light_output_off':   bool(s1 & 0x80),
            'power_5v6_ok':       bool(s0 & 0x01),
        }

    def single_ranging(self) -> dict | None:
        self.send_command(CMD_SINGLE_RANGING)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_SINGLE_RANGING or len(p['params']) != 4:
            return None
        return decode_ranging_response(p['params'])

    def set_target_mode(self, target_mode: int) -> bool:
        if target_mode not in (TARGET_FIRST, TARGET_LAST, TARGET_MULTI):
            return False
        self.send_command(CMD_SET_TARGET_MODE, [target_mode])
        raw = self.read_response()
        if not raw:
            return False
        p = parse_packet(raw)
        return bool(p and p['command_code'] == CMD_SET_TARGET_MODE)

    def start_continuous_ranging(self) -> bool:
        """Send the start-continuous command. Do NOT read the response here –
        the worker thread will read the stream of ranging packets."""
        return self.send_command(CMD_CONTINUE_RANGING)

    def stop_ranging(self) -> bool:
        self.send_command(CMD_STOP_RANGING)
        raw = self.read_response(timeout=1.0)
        if not raw:
            return False
        p = parse_packet(raw)
        return bool(p and p['command_code'] == CMD_STOP_RANGING)

    def set_ranging_frequency(self, frequency: int) -> bool:
        if not 1 <= frequency <= 10:
            raise ValueError("Frequency must be 1–10 Hz")
        self.send_command(CMD_SET_FREQUENCY, [frequency, 0x00])
        raw = self.read_response()
        if not raw:
            return False
        p = parse_packet(raw)
        return bool(p and p['command_code'] == CMD_SET_FREQUENCY)

    def set_min_gating_distance(self, meters: int) -> bool:
        if not 10 <= meters <= 20000:
            raise ValueError("Distance must be 10–20000 m")
        self.send_command(CMD_SET_MIN_DISTANCE, [(meters >> 8) & 0xFF, meters & 0xFF])
        raw = self.read_response()
        if not raw:
            return False
        p = parse_packet(raw)
        return bool(p and p['command_code'] == CMD_SET_MIN_DISTANCE)

    def query_min_gating_distance(self) -> int | None:
        self.send_command(CMD_QUERY_MIN_DISTANCE)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_MIN_DISTANCE or len(p['params']) != 2:
            return None
        return p['params'][0] * 256 + p['params'][1]

    def set_max_gating_distance(self, meters: int) -> bool:
        if not 10 <= meters <= 20000:
            raise ValueError("Distance must be 10–20000 m")
        self.send_command(CMD_SET_MAX_DISTANCE, [(meters >> 8) & 0xFF, meters & 0xFF])
        raw = self.read_response()
        if not raw:
            return False
        p = parse_packet(raw)
        return bool(p and p['command_code'] == CMD_SET_MAX_DISTANCE)

    def query_max_gating_distance(self) -> int | None:
        self.send_command(CMD_QUERY_MAX_DISTANCE)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_MAX_DISTANCE or len(p['params']) != 2:
            return None
        return p['params'][0] * 256 + p['params'][1]

    def query_fpga_version(self) -> dict | None:
        self.send_command(CMD_QUERY_FPGA_VERSION)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_FPGA_VERSION or len(p['params']) != 4:
            return None
        return decode_version_params(p['params'], FPGA_AUTHORS)

    def query_mcu_version(self) -> dict | None:
        self.send_command(CMD_QUERY_MCU_VERSION)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_MCU_VERSION or len(p['params']) != 4:
            return None
        return decode_version_params(p['params'], MCU_AUTHORS)

    def query_hardware_version(self) -> dict | None:
        self.send_command(CMD_QUERY_HARDWARE_VERSION)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_HARDWARE_VERSION or len(p['params']) != 4:
            return None
        mbvs, ctvs, apdvs, ldvs = p['params']
        def _v(b): return f"V{(b>>4)&0xF}.{b&0xF}"
        return {
            'motherboard':    _v(mbvs),
            'control_board':  _v(ctvs),
            'detection_board': _v(apdvs),
            'driver_board':   _v(ldvs),
        }

    def query_sn_number(self) -> dict | None:
        self.send_command(CMD_QUERY_SN_NUMBER)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_SN_NUMBER or len(p['params']) != 3:
            return None
        my, nh, nl = p['params']
        month = (my >> 4) & 0x0F
        year  = 2020 + (my & 0x0F)
        sn    = (nh << 8) | nl
        return {'month': month, 'year': year, 'serial_number': f"{year:04d}{month:02d}{sn:04d}"}

    def query_total_pulses(self) -> int | None:
        self.send_command(CMD_QUERY_TOTAL_PULSES)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_TOTAL_PULSES or len(p['params']) != 3:
            return None
        p3, p2, p1 = p['params']
        return (p3 << 16) | (p2 << 8) | p1

    def query_session_pulses(self) -> int | None:
        self.send_command(CMD_QUERY_SESSION_PULSES)
        raw = self.read_response()
        if not raw:
            return None
        p = parse_packet(raw)
        if not p or p['command_code'] != CMD_QUERY_SESSION_PULSES or len(p['params']) != 3:
            return None
        p3, p2, p1 = p['params']
        return (p3 << 16) | (p2 << 8) | p1