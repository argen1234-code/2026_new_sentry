#!/usr/bin/env python3
"""Keyboard teleop over the ROS graph: publishes geometry_msgs/Twist on /cmd_vel.

Same keys and speed defaults as scripts/chassis_keyboard_teleop.py, but this
variant goes through the stack (cmd_vel_to_unified -> usb_bridge -> STM32)
instead of opening the serial port itself.  Use it whenever usb_bridge owns
/dev/ttyACM* -- e.g. while mapping, where the direct-USB tool would fail.

Keys (hold to move, release to stop):
    W / S   前进 / 后退          (linear.x)
    A / D   左移 / 右移          (linear.y，左为正)
    Q / E   逆时针 / 顺时针自转   (angular.z，REP-103，逆时针为正)
    + / -   所有轴速度整体放大 / 缩小 0.1 倍（0.1x ~ 2.0x，实时生效）
    SPACE   立即停止
    Ctrl-C  停止并退出

Speed defaults match the chassis self-test: vx 0.40, vy 0.30, wz 1.00.
cmd_vel_to_unified clamps to max_vx 0.45 / max_vy 0.30 / max_wz 1.0, so scaling
past 1.0x only has an effect on axes that are still below their clamp.

The node keeps publishing at 50 Hz (zero twist when idle) so the consumer's
command_timeout never fires; on exit it sends a few zero twists first.
"""

import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

PERIOD_S = 0.02          # 50 Hz, same as cmd_vel_to_unified
HOLD_S = 0.60            # a key is considered held while its repeats arrive
STOP_KEY = ' '

DIRECTION_KEYS = {
    'w': ('vx', +1.0),
    's': ('vx', -1.0),
    'a': ('vy', +1.0),
    'd': ('vy', -1.0),
    'q': ('wz', +1.0),   # 逆时针
    'e': ('wz', -1.0),   # 顺时针
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--vx', type=float, default=0.40, help='m/s')
    parser.add_argument('--vy', type=float, default=0.30, help='m/s')
    parser.add_argument('--wz', type=float, default=1.00, help='rad/s')
    parser.add_argument('--hold', type=float, default=HOLD_S,
                        help='seconds a key stays active after its last repeat')
    # Ignore ROS args so `ros2 run`-style remapping does not trip argparse.
    args, _ = parser.parse_known_args(argv)
    return args


def build_twist(active, max_v):
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

    rclpy.init()
    node = Node('keyboard_teleop_ros')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    stdin_fd = sys.stdin.fileno()
    is_tty = sys.stdin.isatty()
    old_term = termios.tcgetattr(stdin_fd) if is_tty else None
    if is_tty:
        tty.setcbreak(stdin_fd)

    def max_v():
        return {k: v * scale for k, v in base_v.items()}

    def show_speed():
        v = max_v()
        print('  [speed] %.1fx  vx=%.2f m/s  vy=%.2f m/s  wz=%.2f rad/s'
              % (scale, v['vx'], v['vy'], v['wz']), flush=True)

    def publish(vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        pub.publish(msg)

    print('ROS keyboard teleop -> /cmd_vel (via cmd_vel_to_unified -> usb_bridge)')
    show_speed()
    print('  按住方向键移动，松开约 %.0f ms 后停；空格立即停，+/- 调速。'
          % (args.hold * 1000))
    print('  Ctrl-C 退出。\n', flush=True)

    active = {}
    last_state = None

    try:
        while rclpy.ok():
            now = time.monotonic()

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

            for key in list(active):
                if now - active[key] > args.hold:
                    del active[key]

            vx, vy, wz, moving = build_twist(active, max_v())
            publish(vx, vy, wz)

            state = (round(vx, 3), round(vy, 3), round(wz, 3))
            if state != last_state:
                print('  [%s] vx=%+.2f vy=%+.2f wz=%+.2f'
                      % ('ENABLED' if moving else 'stop', vx, vy, wz), flush=True)
                last_state = state

            time.sleep(PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            publish(0.0, 0.0, 0.0)
            time.sleep(0.02)
        if old_term is not None:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term)
        print('\n已停止。')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
