#!/usr/bin/env python3
"""Convert /cmd_vel (Twist) into the STM32 UnifiedCmd message.

Motion is disabled by default. When enabled, the message carries
CMD_VALID | CMD_MOTION_ENABLE so the MCU will accept chassis velocities.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sentry_interfaces.msg import UnifiedCmd

from .protocol import CMD_MOTION_ENABLE, CMD_VALID

# ChassisModeNoFollow in stm32_ws/Component/Controller/Chassis/chassis.h.
CHASSIS_MODE_NO_FOLLOW = 2
CHASSIS_MODE_STOP = 0


class CmdVelToUnified(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_unified')

        self.declare_parameter('enable_motion', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_vx', 0.45)
        self.declare_parameter('max_vy', 0.30)
        self.declare_parameter('max_wz', 1.0)
        self.declare_parameter('chassis_mode', CHASSIS_MODE_NO_FOLLOW)

        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.chassis_mode = int(self.get_parameter('chassis_mode').value)

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pub = self.create_publisher(UnifiedCmd, '/unified_cmd', 10)
        self.create_subscription(Twist, topic, self.on_cmd_vel, 10)

        self.get_logger().info(
            f'cmd_vel_to_unified: enable_motion={self.enable_motion} '
            f'topic={topic} limits=({self.max_vx},{self.max_vy},{self.max_wz})')

    @staticmethod
    def _clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def on_cmd_vel(self, msg):
        cmd = UnifiedCmd()

        if not self.enable_motion:
            cmd.chassis_mode = CHASSIS_MODE_STOP
            cmd.chassis_status = 0
            self.pub.publish(cmd)
            return

        vx = self._clamp(msg.linear.x, self.max_vx)
        vy = self._clamp(msg.linear.y, self.max_vy)
        wz = self._clamp(msg.angular.z, self.max_wz)

        cmd.vx_mm_s = int(round(vx * 1000.0))
        cmd.vy_mm_s = int(round(vy * 1000.0))
        cmd.wz_mdeg_s = int(round(wz * 180000.0 / math.pi))
        cmd.chassis_mode = self.chassis_mode
        cmd.chassis_status = CMD_VALID | CMD_MOTION_ENABLE
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUnified()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
