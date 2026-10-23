#!/usr/bin/env python3
"""Keyboard teleop for the chassis_fw firmware (direct USB CDC, no ROS graph).

Keys (hold to move, release to stop):
    W / S   前进 / 后退          (vx)
    A / D   左移 / 右移          (vy，左为正)
    Q / E   逆时针 / 顺时针自转   (wz，主机 REP-103 约定，逆时针为正)
    + / -   所有轴速度整体放大 / 缩小 0.1 倍（0.1x ~ 2.0x，实时生效）
    SPACE   立即停止
    Ctrl-C  停止并退出

Frames are the same UnifiedCmd the ROS usb_bridge sends:
mode=2 (NoFollow), status=CMD_VALID|CMD_MOTION_ENABLE while moving and 0
otherwise, sent at 50 Hz. The firmware stops by itself 100 ms after the last
command, and any exit path here sends status=0 frames first.

The speed defaults are deliberately low for the first on-robot test.

--port defaults to auto: the STM32 CDC is located by VID:PID 0483:5740, the same
rule usb_bridge uses (other CDC devices such as a CMSIS-DAP probe are ignored).
MCU heartbeat/wheel frames are decoded and shown as a [link] line, so the USB
link and the four wheels can be checked without starting ROS.
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'sentry_hardware', 'usb_bridge'))

from usb_bridge.protocol import (  # noqa: E402
    CMD_MOTION_ENABLE,
    CMD_VALID,
    ID_MCU_HEARTBEAT,
    ID_WHEEL_STATE,
    FrameParser,
    ZeroUnifiedCmd,
    decode_mcu_heartbeat,
    decode_wheel_state,
    encode_unified_cmd,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

# Same IDs as usb_bridge_node.py: the chassis MCU enumerates as this CDC.
STM32_VID = 0x0483
STM32_PID = 0x5740

PERIOD_S = 0.02          # 50 Hz, same as cmd_vel_to_unified
HOLD_S = 0.60            # a key is considered held while its repeats arrive
LINK_PRINT_S = 0.50      # how often the downlink state line is refreshed
STOP_KEY = ' '
MODE_NO_FOLLOW = 2

DIRECTION_KEYS = {
    'w': ('vx', +1.0),
    's': ('vx', -1.0),
    'a': ('vy', +1.0),
    'd': ('vy', -1.0),
    'q': ('wz', +1.0),   # 逆时针
    'e': ('wz', -1.0),   # 顺时针
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='auto',
                        help="serial device or 'auto' (STM32 VID:PID 0483:5740)")
    parser.add_argument('--vx', type=float, default=0.40, help='m/s')
    parser.add_argument('--vy', type=float, default=0.30, help='m/s')
    parser.add_argument('--wz', type=float, default=1.00, help='rad/s')
    parser.add_argument('--hold', type=float, default=HOLD_S,
                        help='seconds a key stays active after its last repeat')
    parser.add_argument('--dry-run', action='store_true',
                        help='do not open the serial port; print the command')
    parser.add_argument('--list-ports', action='store_true',
                        help='list serial ports and exit')
    return parser.parse_args()


def list_serial_ports():
    for port in list_ports.comports():
        vid = '%04X' % port.vid if port.vid is not None else '----'
        pid = '%04X' % port.pid if port.pid is not None else '----'
        print('  %-16s %s:%s  %s' % (port.device, vid, pid, port.description))


def resolve_port(value):
    """Return the device path; 'auto' picks the STM32 CDC by VID:PID."""
    if value.lower() != 'auto':
        return value
    candidates = [
        port for port in list_ports.comports()
        if port.vid == STM32_VID and port.pid == STM32_PID
    ]
    if not candidates:
        print('未找到 STM32 USB CDC（VID:PID %04X:%04X），当前串口：' %
              (STM32_VID, STM32_PID), file=sys.stderr)
        list_serial_ports()
        sys.exit(1)
    return candidates[0].device


def build_command(active, max_v):
    """active: {key: last_seen_s}. Returns (vx, vy, wz, moving)."""
    vx = vy = wz = 0.0
    moving = False
    for key in active:
        axis, sign = DIRECTION_KEYS[key]
        if axis == 'vx':
            vx += sign * max_v['vx']
        elif axis == 'vy':
            vy += sign * max_v['vy']
        else:
            wz += sign * max_v['wz']
        moving = True
    return vx, vy, wz, moving


def main():
    args = parse_args()
    base_v = {'vx': abs(args.vx), 'vy': abs(args.vy), 'wz': abs(args.wz)}
    scale = 1.0

    if args.list_ports:
        if list_ports is None:
            sys.exit('pyserial is required to list ports')
        list_serial_ports()
        return

    ser = None
    if not args.dry_run:
        if serial is None:
            sys.exit('pyserial is required unless --dry-run is used')
        port = resolve_port(args.port)
        try:
            ser = serial.Serial(port, 115200, timeout=0)
        except serial.SerialException as exc:
            sys.exit('无法打开 %s：%s\n（usb_bridge 节点是否仍在运行？）' %
                     (port, exc))
        ser.reset_input_buffer()
        print('  端口: %s' % port)
        time.sleep(0.2)

    stdin_fd = sys.stdin.fileno()
    is_tty = sys.stdin.isatty()
    old_term = termios.tcgetattr(stdin_fd) if is_tty else None
    if is_tty:
        tty.setcbreak(stdin_fd)

    active = {}
    seq = 0
    last_status = None

    parser = FrameParser()
    rx_frames = 0
    hb = wheel = None
    hb_at = wheel_at = 0.0
    last_link_print = 0.0

    def send(vx, vy, wz, moving):
        nonlocal seq
        status = (CMD_VALID | CMD_MOTION_ENABLE) if moving else 0
        cmd = ZeroUnifiedCmd(
            vx_mm_s=int(round(vx * 1000.0)),
            vy_mm_s=int(round(vy * 1000.0)),
            wz_mdeg_s=int(round(wz * 180000.0 / math.pi)),
            chassis_mode=MODE_NO_FOLLOW,
            chassis_status=status,
            host_ts_ns=time.time_ns(),
        )
        frame = encode_unified_cmd(seq, cmd, host_ts_ns=time.time_ns())
        seq = (seq + 1) & 0xFF
        if ser is not None:
            ser.write(frame)
        return status, frame

    def drain_uplink(now):
        """Read MCU frames without blocking; keep the latest state."""
        nonlocal rx_frames, hb, wheel, hb_at, wheel_at
        chunk = ser.read(256)
        if not chunk:
            return
        for frame in parser.feed(chunk):
            rx_time = time.monotonic()
            try:
                if frame.msg_id == ID_MCU_HEARTBEAT:
                    hb = decode_mcu_heartbeat(frame.payload)
                    hb_at = rx_time
                elif frame.msg_id == ID_WHEEL_STATE:
                    wheel = decode_wheel_state(frame.payload)
                    wheel_at = rx_time
                else:
                    continue
            except ValueError:
                parser.errors += 1
                continue
            rx_frames += 1

    def link_line(now):
        parts = []
        if hb is not None:
            parts.append('hb %3.0fms mode=%d status=0x%02X err=0x%04X' % (
                (now - hb_at) * 1000.0, hb['mode'], hb['status'],
                hb['error_flags']))
        if wheel is not None:
            speeds = ' '.join('%+.2f' % v for v in wheel['speed_mps'])
            parts.append('wheel %3.0fms online=0x%X fault=0x%04X v=[%s]' % (
                (now - wheel_at) * 1000.0, wheel['online_mask'],
                wheel['fault_flags'], speeds))
        if not parts:
            parts.append('无上行帧')
        parts.append('rx=%d err=%d' % (rx_frames, parser.errors))
        print('  [link] ' + '  '.join(parts))

    def max_v():
        return {k: v * scale for k, v in base_v.items()}

    def show_speed():
        v = max_v()
        print('  [speed] %.1fx  vx=%.2f m/s  vy=%.2f m/s  wz=%.2f rad/s' %
              (scale, v['vx'], v['vy'], v['wz']))

    print(__doc__.split('Keys (hold to move, release to stop):')[0].strip())
    show_speed()
    print('  按住方向键移动，松开约 %.0f ms 后停；空格立即停，+/- 调速。' %
          (args.hold * 1000))
    print('  Ctrl-C 退出。\n')

    try:
        while True:
            now = time.monotonic()

            # Drain available input; the terminal delivers a repeat while held.
            while True:
                ready, _, _ = select.select([stdin_fd], [], [], 0)
                if not ready:
                    break
                data = os.read(stdin_fd, 64)
                if not data:
                    break
                for ch in data.decode(errors='ignore'):
                    key = ch.lower()
                    if key in DIRECTION_KEYS:
                        active[key] = now
                    elif key == STOP_KEY:
                        active.clear()
                    elif key in ('+', '='):
                        scale = min(2.0, round(scale + 0.1, 1))
                        show_speed()
                    elif key == '-':
                        scale = max(0.1, round(scale - 0.1, 1))
                        show_speed()

            # Drop keys whose auto-repeat stopped (release) or that timed out.
            for key in list(active):
                if now - active[key] > args.hold:
                    del active[key]

            vx, vy, wz, moving = build_command(active, max_v())
            status, frame = send(vx, vy, wz, moving)

            if status != last_status:
                state = 'ENABLED' if moving else 'stop'
                print('  [%s] vx=%+.2f vy=%+.2f wz=%+.2f' % (state, vx, vy, wz))
                if args.dry_run:
                    print('    frame: %s' % frame.hex(' '))
                last_status = status

            if ser is not None:
                drain_uplink(now)
                if now - last_link_print >= LINK_PRINT_S:
                    last_link_print = now
                    link_line(now)

            time.sleep(PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit stop: disabled status, zero velocity, sent a few times.
        for _ in range(10):
            send(0.0, 0.0, 0.0, False)
            if ser is not None:
                ser.flush()
            time.sleep(0.02)
        if ser is not None:
            ser.close()
        if old_term is not None:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term)
        print('\n已停止。')


if __name__ == '__main__':
    main()
