import struct

import pytest

from usb_bridge.protocol import (
    CMD_MOTION_ENABLE,
    CMD_VALID,
    IMU_STATE_FMT,
    IMU_STATE_LEN,
    MCU_STATE_LEN,
    UNIFIED_CMD_LEN,
    ODOM_STATE_FMT,
    ODOM_STATE_LEN,
    WHEEL_STATE_FMT,
    WHEEL_STATE_LEN,
    FrameParser,
    decode_imu_state,
    decode_odom_state,
    decode_wheel_state,
    encode_frame,
)


def test_lengths_are_stable():
    assert UNIFIED_CMD_LEN == 45
    assert MCU_STATE_LEN == 40
    assert WHEEL_STATE_LEN == 58
    assert IMU_STATE_LEN == 58
    assert ODOM_STATE_LEN == 36
    assert struct.calcsize(WHEEL_STATE_FMT) == 58
    assert struct.calcsize(IMU_STATE_FMT) == 58
    assert struct.calcsize(ODOM_STATE_FMT) == 36


def test_command_bits():
    assert CMD_VALID == 0x0001
    assert CMD_MOTION_ENABLE == 0x0002


def test_decode_wheel_state_roundtrip():
    payload = struct.pack(
        WHEEL_STATE_FMT,
        1234, 0x0F, 2, 0, 0x0005,
        0.1, -0.2, 0.3, -0.4,
        1.0, -2.0, 3.0, -4.0,
        31, 32, 33, 34,
        41, 42, 43, 44,
        100, -200, 300, -400,
    )
    state = decode_wheel_state(payload)
    assert state['mcu_ts_ms'] == 1234
    assert state['online_mask'] == 0x0F
    assert state['mode'] == 2
    assert state['fault_flags'] == 5
    assert state['speed_mps'] == pytest.approx([0.1, -0.2, 0.3, -0.4])
    assert state['position_rad'] == pytest.approx([1.0, -2.0, 3.0, -4.0])
    assert state['temp_mos_c'] == [31, 32, 33, 34]
    assert state['temp_motor_c'] == [41, 42, 43, 44]
    assert state['torque_mnm'] == [100, -200, 300, -400]


def test_decode_imu_state_roundtrip():
    payload = struct.pack(
        IMU_STATE_FMT,
        5678,
        0.01, 0.02, 0.03,
        0.0, 0.0, 9.81,
        35.5,
        1.0, 0.0, 0.0, 0.0,
        0,
        0x07,
        0,
        9999,
    )
    state = decode_imu_state(payload)
    assert state['mcu_ts_ms'] == 5678
    assert state['gyro'] == pytest.approx([0.01, 0.02, 0.03])
    assert state['accel'] == pytest.approx([0.0, 0.0, 9.81])
    assert state['temperature_c'] == pytest.approx(35.5)
    assert state['quat_wxyz'] == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert state['valid_mask'] == 0x07
    assert state['last_ok_ms'] == 9999


def test_decode_odom_state_roundtrip():
    payload = struct.pack(
        ODOM_STATE_FMT,
        4321,
        0.15, -0.20, 0.35, -0.05, 0.40, -0.10,
        987654,
        21,
        1,
        0,
    )
    state = decode_odom_state(payload)
    assert state['mcu_ts_ms'] == 4321
    assert state['x_m'] == pytest.approx(0.15)
    assert state['y_m'] == pytest.approx(-0.20)
    assert state['yaw_rad'] == pytest.approx(0.35)
    assert state['gyro_rad_s'] == pytest.approx(-0.05)
    assert state['vx_mps'] == pytest.approx(0.40)
    assert state['vy_mps'] == pytest.approx(-0.10)
    assert state['odom_seq'] == 987654
    assert state['age_ms'] == 21
    assert state['valid'] is True


def test_parser_reads_odom_frame():
    payload = bytes(ODOM_STATE_LEN)
    frame = encode_frame(0x85, 9, payload)
    parser = FrameParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    assert frames[0].msg_id == 0x85
    assert frames[0].seq == 9


def test_parser_reads_wheel_frame():
    payload = bytes(WHEEL_STATE_LEN)
    frame = encode_frame(0x83, 7, payload)
    parser = FrameParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    assert frames[0].msg_id == 0x83
    assert frames[0].seq == 7
    assert frames[0].payload == payload
