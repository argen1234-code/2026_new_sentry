#!/usr/bin/env python3
"""Jetson <-> STM32 USB CDC bridge node.

TX: HOST_HEARTBEAT at 10 Hz and UNIFIED_CMD at 50 Hz (zero-safe when stale).
RX: MCU_STATE at 50 Hz -> /usb_bridge/state, MCU_HEARTBEAT for link detection.

The STM32 enumerates as USB CDC with VID:PID 0483:5740. The node auto-detects
that port so it never opens the DAPLink or the LiDAR serial port by mistake.
"""

import time

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from sentry_interfaces.msg import ChassisState, UnifiedCmd, UsbBridgeState

import serial
from serial.tools import list_ports

from .protocol import (
    FrameParser,
    ID_IMU_STATE,
    ID_MCU_HEARTBEAT,
    ID_MCU_STATE,
    ID_WHEEL_STATE,
    ZeroUnifiedCmd,
    decode_imu_state,
    decode_mcu_state,
    decode_wheel_state,
    encode_host_heartbeat,
    encode_unified_cmd,
)

STM32_VID = 0x0483
STM32_PID = 0x5740

# MCU array order is RR, FL, RL, FR. Joint signs convert motor-positive
# feedback to forward-positive wheel rotation (URDF axis +Y).
WHEEL_JOINTS = [
    'wheel_back_right_joint',
    'wheel_front_left_joint',
    'wheel_back_left_joint',
    'wheel_front_right_joint',
]
WHEEL_JOINT_SIGNS = [-1.0, 1.0, 1.0, -1.0]
WHEEL_RADIUS_M = 0.045
WHEEL_POSITION_WRAP_RAD = 25.0


class UsbBridge(Node):
    def __init__(self):
        super().__init__('usb_bridge')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('enabled', True)
        self.declare_parameter('command_timeout', 0.4)
        self.declare_parameter('tx_period', 0.02)
        self.declare_parameter('poll_period', 0.005)
        self.declare_parameter('link_timeout', 1.0)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS_M)
        self.declare_parameter('wheel_position_wrap_rad', WHEEL_POSITION_WRAP_RAD)
        self.declare_parameter('wheel_joint_signs', WHEEL_JOINT_SIGNS)

        self.port_param = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.tx_period = float(self.get_parameter('tx_period').value)
        self.poll_period = float(self.get_parameter('poll_period').value)
        self.link_timeout = float(self.get_parameter('link_timeout').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_position_wrap = float(
            self.get_parameter('wheel_position_wrap_rad').value)
        self.wheel_signs = [float(v) for v in
                            self.get_parameter('wheel_joint_signs').value]

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

        # Dynamic state and per-wheel position unwrapping.
        self.wheel_state = None
        self.imu_state = None
        self.last_wheel_seq = None
        self.last_imu_seq = None
        self.wheel_seq_gaps = 0
        self.imu_seq_gaps = 0
        self.wheel_last_raw = [None] * 4
        self.wheel_unwrapped = [0.0] * 4

        self.state_pub = self.create_publisher(UsbBridgeState, '/usb_bridge/state', 10)
        self.chassis_pub = self.create_publisher(ChassisState, '/chassis/state', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.joint_pub = self.create_publisher(JointState, 'serial/joint_states_raw', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
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
            elif frame.msg_id == ID_WHEEL_STATE:
                self.rx_frames += 1
                try:
                    state = decode_wheel_state(frame.payload)
                except ValueError as exc:
                    self.get_logger().warn(f'Bad wheel state frame: {exc}',
                                           throttle_duration_sec=2.0)
                    continue
                self._track_seq(frame.seq, 'last_wheel_seq', 'wheel_seq_gaps')
                self.wheel_state = state
                self.publish_wheel_state(frame.seq, state)
            elif frame.msg_id == ID_IMU_STATE:
                self.rx_frames += 1
                try:
                    state = decode_imu_state(frame.payload)
                except ValueError as exc:
                    self.get_logger().warn(f'Bad IMU state frame: {exc}',
                                           throttle_duration_sec=2.0)
                    continue
                self._track_seq(frame.seq, 'last_imu_seq', 'imu_seq_gaps')
                self.imu_state = state
                self.publish_imu_state(state)
            elif frame.msg_id == ID_MCU_HEARTBEAT:
                self.rx_frames += 1
            else:
                self.unknown_frames += 1

    def _track_seq(self, seq, last_attr, gap_attr):
        last = getattr(self, last_attr)
        if last is not None:
            delta = (seq - last) & 0xFF
            if delta != 1:
                setattr(self, gap_attr, getattr(self, gap_attr) + 1)
        setattr(self, last_attr, seq)

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

    def _unwrap_wheel(self, index, raw):
        last = self.wheel_last_raw[index]
        if last is None or self.wheel_position_wrap <= 0.0:
            self.wheel_last_raw[index] = raw
            self.wheel_unwrapped[index] = raw
            return raw

        delta = raw - last
        half = self.wheel_position_wrap * 0.5
        if delta > half:
            delta -= self.wheel_position_wrap
        elif delta < -half:
            delta += self.wheel_position_wrap

        self.wheel_unwrapped[index] += delta
        self.wheel_last_raw[index] = raw
        return self.wheel_unwrapped[index]

    def publish_wheel_state(self, seq, state):
        now = self.get_clock().now().to_msg()

        msg = ChassisState()
        msg.header.stamp = now
        msg.header.frame_id = 'base_link'
        msg.seq = seq
        msg.mcu_ts_ms = state['mcu_ts_ms']
        msg.online_mask = state['online_mask']
        msg.mode = state['mode']
        msg.chassis_status = state['chassis_status']
        msg.fault_flags = state['fault_flags']
        msg.speed_mps = state['speed_mps']
        msg.position_rad = state['position_rad']
        msg.torque_nm = [float(v) / 1000.0 for v in state['torque_mnm']]
        msg.temp_mos_c = state['temp_mos_c']
        msg.temp_motor_c = state['temp_motor_c']
        self.chassis_pub.publish(msg)

        js = JointState()
        js.header.stamp = now
        js.name = list(WHEEL_JOINTS)
        js.position = []
        js.velocity = []
        for i, sign in enumerate(self.wheel_signs):
            unwrapped = self._unwrap_wheel(i, state['position_rad'][i])
            js.position.append(sign * unwrapped)
            js.velocity.append(sign * state['speed_mps'][i] / self.wheel_radius)
        self.joint_pub.publish(js)

    def publish_imu_state(self, state):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        if state['valid_mask'] & 0x02:
            q = state['quat_wxyz']
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0

        if state['valid_mask'] & 0x01:
            msg.angular_velocity.x = state['gyro'][0]
            msg.angular_velocity.y = state['gyro'][1]
            msg.angular_velocity.z = state['gyro'][2]
            msg.linear_acceleration.x = state['accel'][0]
            msg.linear_acceleration.y = state['accel'][1]
            msg.linear_acceleration.z = state['accel'][2]
        else:
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'usb_bridge: STM32 link'
        status.hardware_id = 'stm32'
        if self.ser is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'serial port not open'
        elif not self.link_online:
            status.level = DiagnosticStatus.WARN
            status.message = 'link timeout'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'ok'

        values = {
            'port': self.port_name or '',
            'link_online': str(self.link_online),
            'rx_frames': str(self.rx_frames),
            'tx_frames': str(self.tx_frames),
            'tx_errors': str(self.tx_errors),
            'parser_errors': str(self.parser.errors),
            'unknown_frames': str(self.unknown_frames),
            'wheel_seq_gaps': str(self.wheel_seq_gaps),
            'imu_seq_gaps': str(self.imu_seq_gaps),
        }
        if self.wheel_state is not None:
            values['online_mask'] = str(self.wheel_state['online_mask'])
            values['fault_flags'] = str(self.wheel_state['fault_flags'])
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        arr.status.append(status)
        self.diag_pub.publish(arr)

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
            self.publish_offline_state()

        if self.ser is None:
            self.publish_offline_state()
            self.publish_diagnostics()
            return

        self.publish_diagnostics()
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
