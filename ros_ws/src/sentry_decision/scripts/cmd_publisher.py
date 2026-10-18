#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sentry_decision.msg import ExtraCmd

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.pub = self.create_publisher(ExtraCmd, '/cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.msg = ExtraCmd()
        self.msg.dm_extra_vx = 10
        self.msg.dm_extra_vy = 12
        self.msg.dm_extra_wz = 15
        self.msg.dm_extra_flag_wz = 1
        self.get_logger().info('Publishing /cmd: vx=10, vy=12, wz=15, flag=1')

    def timer_callback(self):
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
