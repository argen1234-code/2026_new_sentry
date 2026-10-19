"""Jetson <-> STM32 USB CDC frame protocol.

Frame: AA 55 | id | seq | len | payload[len] | xor
xor = id ^ seq ^ len ^ payload[0] ^ ... ^ payload[len-1]
All multi-byte fields are little-endian.
"""

from dataclasses import dataclass
import struct

SOF0 = 0xAA
SOF1 = 0x55
MAX_PAYLOAD = 64

ID_HOST_HEARTBEAT = 0x01
ID_UNIFIED_CMD = 0x10
ID_MCU_STATE = 0x81
ID_MCU_HEARTBEAT = 0x82

# Keep these layouts in sync with
# stm32_ws/Component/Communication/Src/communication_usb.h.
UNIFIED_CMD_FMT = '<iiHiiiiHBiiQ'
MCU_STATE_FMT = '<I8fBBH'
MCU_HEARTBEAT_FMT = '<IBBH'

UNIFIED_CMD_LEN = struct.calcsize(UNIFIED_CMD_FMT)
MCU_STATE_LEN = struct.calcsize(MCU_STATE_FMT)
MCU_HEARTBEAT_LEN = struct.calcsize(MCU_HEARTBEAT_FMT)

assert UNIFIED_CMD_LEN == 45
assert MCU_STATE_LEN == 40
assert MCU_HEARTBEAT_LEN == 8


@dataclass(frozen=True)
class Frame:
    msg_id: int
    seq: int
    payload: bytes


@dataclass
class ZeroUnifiedCmd:
    """Fail-safe command used when no fresh UnifiedCmd is available."""

    delta_yaw_udeg: int = 0
    delta_pitch_udeg: int = 0
    gimbal_status: int = 0
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    wz_mdeg_s: int = 0
    chassis_mode: int = 0
    chassis_status: int = 0
    fire_on: bool = False
    fire_mode: int = 0
    burst_count: int = 0
    host_ts_ns: int = 0


def xor_checksum(msg_id, seq, payload):
    value = (msg_id ^ seq ^ len(payload)) & 0xFF
    for byte in payload:
        value ^= byte
    return value


def encode_frame(msg_id, seq, payload):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload too long: {len(payload)}')
    return bytes((SOF0, SOF1, msg_id & 0xFF, seq & 0xFF, len(payload))) + payload + bytes(
        (xor_checksum(msg_id, seq, payload),)
    )


def encode_host_heartbeat(seq, mode=3, flags=0):
    payload = struct.pack('<BB', mode & 0xFF, flags & 0xFF)
    return encode_frame(ID_HOST_HEARTBEAT, seq, payload)


def encode_unified_cmd(seq, cmd, host_ts_ns=None):
    host_ts = cmd.host_ts_ns if host_ts_ns is None else host_ts_ns
    payload = struct.pack(
        UNIFIED_CMD_FMT,
        int(cmd.delta_yaw_udeg),
        int(cmd.delta_pitch_udeg),
        int(cmd.gimbal_status),
        int(cmd.vx_mm_s),
        int(cmd.vy_mm_s),
        int(cmd.wz_mdeg_s),
        int(cmd.chassis_mode),
        int(cmd.chassis_status),
        1 if cmd.fire_on else 0,
        int(cmd.fire_mode),
        int(cmd.burst_count),
        int(host_ts),
    )
    return encode_frame(ID_UNIFIED_CMD, seq, payload)


def decode_mcu_state(payload):
    if len(payload) != MCU_STATE_LEN:
        raise ValueError(f'mcu state length {len(payload)} != {MCU_STATE_LEN}')
    values = struct.unpack(MCU_STATE_FMT, payload)
    return {
        'mcu_ts_ms': values[0],
        'imu_roll': values[1],
        'imu_pitch': values[2],
        'imu_yaw': values[3],
        'gimbal_yaw': values[4],
        'gimbal_pitch': values[5],
        'vx': values[6],
        'vy': values[7],
        'wz': values[8],
        'mode': values[9],
        'status': values[10],
        'error_flags': values[11],
    }


def decode_mcu_heartbeat(payload):
    if len(payload) != MCU_HEARTBEAT_LEN:
        raise ValueError(f'mcu heartbeat length {len(payload)} != {MCU_HEARTBEAT_LEN}')
    ts, mode, status, error_flags = struct.unpack(MCU_HEARTBEAT_FMT, payload)
    return {
        'mcu_ts_ms': ts,
        'mode': mode,
        'status': status,
        'error_flags': error_flags,
    }


class FrameParser:
    """Incremental parser; feed arbitrary byte chunks and get complete frames."""

    def __init__(self):
        self._buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        if data:
            self._buffer.extend(data)

        frames = []
        while True:
            # Drop bytes until a possible start-of-frame.
            while self._buffer and self._buffer[0] != SOF0:
                del self._buffer[0]
            if len(self._buffer) < 5:
                break
            if self._buffer[1] != SOF1:
                del self._buffer[0]
                continue

            msg_id = self._buffer[2]
            seq = self._buffer[3]
            length = self._buffer[4]
            if length > MAX_PAYLOAD:
                self.errors += 1
                del self._buffer[0]
                continue

            total = 6 + length
            if len(self._buffer) < total:
                break

            payload = bytes(self._buffer[5:5 + length])
            checksum = self._buffer[5 + length]
            if checksum != xor_checksum(msg_id, seq, payload):
                self.errors += 1
                del self._buffer[0]
                continue

            frames.append(Frame(msg_id, seq, payload))
            del self._buffer[:total]

        return frames
