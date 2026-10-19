#!/usr/bin/env python3
"""Jetson <-> STM32 USB CDC bridge node.

TX: HOST_HEARTBEAT at 10 Hz and UNIFIED_CMD at 50 Hz (zero-safe when stale).
RX: MCU_STATE at 50 Hz -> /usb_bridge/state, MCU_HEARTBEAT for link detection.

The STM32 enumerates as USB CDC with VID:PID 0483:5740. The node auto-detects
that port so it never opens the DAPLink or the LiDAR serial port by mistake.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sentry_interfaces.msg import UnifiedCmd, UsbBridgeState

import serial
from serial.tools import list_ports

from .protocol import (
    FrameParser,
    ID_MCU_HEARTBEAT,
    ID_MCU_STATE,
    ZeroUnifiedCmd,
    decode_mcu_state,
    encode_host_heartbeat,
    encode_unified_cmd,
)

STM32_VID = 0x0483
STM32_PID = 0x5740


class UsbBridge(Node):
    def __init__(self):
        super().__init__('usb_bridge')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('enabled', True)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('tx_period', 0.02)
        self.declare_parameter('poll_period', 0.005)
        self.declare_parameter('link_timeout', 1.0)

        self.port_param = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.tx_period = float(self.get_parameter('tx_period').value)
        self.poll_period = float(self.get_parameter('poll_period').value)
        self.link_timeout = float(self.get_parameter('link_timeout').value)

        self.ser = None
        self.port_name = None
        self.parser = FrameParser()
        self.seq = 0
        self.last_open_attempt = 0.0
        self.last_heartbeat = 0.0
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.last_rx_time = 0.0
        self.link_online = False
        self.rx_frames = 0
        self.unknown_frames = 0
        self.tx_frames = 0
        self.tx_errors = 0

        self.state_pub = self.create_publisher(UsbBridgeState, '/usb_bridge/state', 10)
        self.create_subscription(UnifiedCmd, '/unified_cmd', self.on_unified_cmd, 10)

        self.create_timer(self.poll_period, self.poll_rx)
        self.create_timer(self.tx_period, self.tick_tx)
        self.create_timer(1.0, self.report)

        if not self.enabled:
            self.get_logger().warn('USB bridge disabled (enabled:=false), monitor only')

    # ------------------------------------------------------------------
    # Port handling
    # ------------------------------------------------------------------
    def _detect_port(self):
        if self.port_param and self.port_param.lower() != 'auto':
            return self.port_param

        fallback = None
        for candidate in list_ports.comports():
            if candidate.vid == STM32_VID and candidate.pid == STM32_PID:
                return candidate.device
            if candidate.description and 'Printk DM 02' in candidate.description:
                fallback = candidate.device
        return fallback

    def _open(self):
        if not self.enabled:
            return False

        now = time.monotonic()
        if now - self.last_open_attempt < 1.0:
            return False
        self.last_open_attempt = now

        device = self._detect_port()
        if not device:
            self.get_logger().warn(
                'STM32 USB CDC not found (VID:PID 0483:5740); retrying',
                throttle_duration_sec=5.0)
            return False

        try:
            self.ser = serial.Serial(
                device,
                self.baud,
                timeout=0,
                write_timeout=0.05,
                exclusive=False,
            )
            self.ser.dtr = True
            self.ser.reset_input_buffer()
            self.port_name = device
            self.last_rx_time = 0.0
            self.link_online = False
            self.get_logger().info(f'STM32 USB CDC connected: {device}')
            return True
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Cannot open STM32 USB CDC {device}: {exc}',
                                    throttle_duration_sec=5.0)
            self.ser = None
            return False

    def _close(self, reason=''):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None
        self.port_name = None
        self.link_online = False
        if reason:
            self.get_logger().warn(f'USB bridge disconnected: {reason}')

    def _write(self, frame):
        if self.ser is None:
            return False
        try:
            self.ser.write(frame)
            self.tx_frames += 1
            return True
        except (serial.SerialException, OSError) as exc:
            self.tx_errors += 1
            self._close(f'write failed: {exc}')
            return False

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def on_unified_cmd(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    # ------------------------------------------------------------------
    # RX / TX timers
    # ------------------------------------------------------------------
    def poll_rx(self):
        if self.ser is None:
            self._open()
            return

        try:
            waiting = self.ser.in_waiting
            data = self.ser.read(waiting) if waiting else b''
        except (serial.SerialException, OSError) as exc:
            self._close(f'read failed: {exc}')
            return

        if not data:
            return

        now = time.monotonic()
        self.last_rx_time = now
        self.link_online = True

        for frame in self.parser.feed(data):
            if frame.msg_id == ID_MCU_STATE:
                self.rx_frames += 1
                try:
                    state = decode_mcu_state(frame.payload)
                except ValueError as exc:
                    self.get_logger().warn(f'Bad MCU state frame: {exc}',
                                           throttle_duration_sec=2.0)
                    continue
                self.publish_state(state)
            elif frame.msg_id == ID_MCU_HEARTBEAT:
                self.rx_frames += 1
            else:
                self.unknown_frames += 1

    def tick_tx(self):
        if not self.enabled:
            return
        if self.ser is None:
            return

        now = time.monotonic()

        if now - self.last_heartbeat >= 0.1:
            self.last_heartbeat = now
            self._write(encode_host_heartbeat(self._next_seq(), mode=3, flags=0))

        if self.last_cmd is not None and (now - self.last_cmd_time) <= self.command_timeout:
            command = self.last_cmd
        else:
            command = ZeroUnifiedCmd()

        frame = encode_unified_cmd(
            self._next_seq(), command,
            host_ts_ns=self.get_clock().now().nanoseconds)
        self._write(frame)

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    # ------------------------------------------------------------------
    # State publishing / diagnostics
    # ------------------------------------------------------------------
    def publish_state(self, state):
        msg = UsbBridgeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'stm32'
        msg.mcu_ts_ms = state['mcu_ts_ms']
        msg.imu_roll = state['imu_roll']
        msg.imu_pitch = state['imu_pitch']
        msg.imu_yaw = state['imu_yaw']
        msg.gimbal_yaw = state['gimbal_yaw']
        msg.gimbal_pitch = state['gimbal_pitch']
        msg.vx = state['vx']
        msg.vy = state['vy']
        msg.wz = state['wz']
        msg.mode = state['mode']
        msg.status = state['status']
        msg.error_flags = state['error_flags']
        msg.link_online = self.link_online
        msg.rx_frames = self.rx_frames
        msg.crc_errors = self.parser.errors
        self.state_pub.publish(msg)

    def publish_offline_state(self):
        msg = UsbBridgeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'stm32'
        msg.link_online = False
        msg.rx_frames = self.rx_frames
        msg.crc_errors = self.parser.errors
        self.state_pub.publish(msg)

    def report(self):
        now = time.monotonic()
        if self.link_online and (now - self.last_rx_time) > self.link_timeout:
            self.link_online = False
            self.get_logger().warn('STM32 USB link timeout')

        if self.ser is None:
            self.publish_offline_state()
            return

        self.get_logger().info(
            f'port={self.port_name} link={self.link_online} '
            f'rx={self.rx_frames} tx={self.tx_frames} '
            f'parser_errors={self.parser.errors} tx_errors={self.tx_errors}')

    def shutdown(self):
        if self.ser is not None and self.enabled:
            for _ in range(3):
                self._write(encode_unified_cmd(
                    self._next_seq(), ZeroUnifiedCmd(),
                    host_ts_ns=self.get_clock().now().nanoseconds))
                time.sleep(0.02)
        self._close()


def main(args=None):
    rclpy.init(args=args)
    node = UsbBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
