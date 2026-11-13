#!/usr/bin/env python3
"""定位安全监督：定位状态异常持续超过阈值时，取消导航目标让车停下。

对齐东北大学 TDT 的原则：定位失败要可检测、输出要有界。
- 订阅 /diagnostics 里 small_gicp 的 localization/gicp 状态；
- 状态为 OBS_WEAK（观测覆盖不足）或 DEGRADED（配准冻结）持续 > hold_s 秒，
  调用 navigate_to_pose 的 cancel_goal 服务取消所有目标；
- 恢复 NORMAL 后不自动重发目标，由操作手/上层决定。
"""
import rclpy
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID


class LocalizationSupervisor(Node):
    def __init__(self):
        super().__init__('localization_supervisor')
        self.hold_s = float(self.declare_parameter('hold_s', 2.0).value)
        self.watch_names = list(
            self.declare_parameter(
                'watch_names', ['localization/gicp']).value)
        self.abnormal_states = list(
            self.declare_parameter(
                'abnormal_states', ['OBS_WEAK', 'DEGRADED']).value)
        self.cancel_service = str(
            self.declare_parameter(
                'cancel_service', '/navigate_to_pose/_action/cancel_goal').value)
        self.enabled = bool(self.declare_parameter('enabled', True).value)

        self.state = 'NORMAL'
        self.abnormal_since = None
        self.cancelled = False

        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.cancel_cli = self.create_client(CancelGoal, self.cancel_service)
        self.create_timer(0.2, self.check)
        self.get_logger().info(
            f'localization_supervisor 启动：hold={self.hold_s:.1f}s, '
            f'abnormal={self.abnormal_states}')

    def on_diagnostics(self, msg: DiagnosticArray):
        for status in msg.status:
            if status.name in self.watch_names:
                kv = {item.key: item.value for item in status.values}
                self.state = kv.get('state', status.message or 'NORMAL')

    def cancel_all_goals(self):
        if not self.cancel_cli.service_is_ready():
            self.get_logger().warn('cancel_goal 服务不可用，无法取消目标')
            return
        req = CancelGoal.Request()
        req.goal_info.goal_id = UUID(uuid=[0] * 16)
        req.goal_info.stamp.sec = 0
        req.goal_info.stamp.nanosec = 0
        self.cancel_cli.call_async(req)
        self.get_logger().error(
            f'定位状态 {self.state} 持续 {self.hold_s:.1f}s，已取消导航目标并停车')

    def check(self):
        if not self.enabled:
            return
        abnormal = self.state in self.abnormal_states
        now = self.get_clock().now().nanoseconds / 1e9
        if abnormal:
            if self.abnormal_since is None:
                self.abnormal_since = now
                self.get_logger().warn(f'定位进入异常状态 {self.state}，开始计时')
            elif not self.cancelled and now - self.abnormal_since >= self.hold_s:
                self.cancel_all_goals()
                self.cancelled = True
        else:
            if self.abnormal_since is not None:
                self.get_logger().info('定位恢复 NORMAL')
            self.abnormal_since = None
            self.cancelled = False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
