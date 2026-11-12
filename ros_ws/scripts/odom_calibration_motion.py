#!/usr/bin/env python3
"""轮式里程计标定运动序列（发 /cmd_vel，经 cmd_vel_to_unified 下发）。

序列固定、可重复，便于按时间段切分数据后计算：
  - 平移段：/wheel_odom 与 /lidar_odometry 的位移夹角 -> 底盘系安装角
  - 平移段：位移模长比 -> 尺度
  - 自转段：yaw 变化比 -> yaw 符号与尺度

车体着地且周围有障碍，所以每段位移不超过约 0.30 m，自转约 45°。
运行前必须回车确认，Ctrl-C 立即发零速退出。
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_VEL_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 20.0

# (段名, vx m/s, vy m/s, wz rad/s, 持续秒数)
SEGMENTS = [
    ('static_a', 0.0, 0.0, 0.0, 3.0),
    ('x_pos', 0.40, 0.0, 0.0, 0.75),
    ('pause1', 0.0, 0.0, 0.0, 2.0),
    ('x_neg', -0.40, 0.0, 0.0, 0.75),
    ('pause2', 0.0, 0.0, 0.0, 2.0),
    ('y_pos', 0.0, 0.40, 0.0, 0.75),
    ('pause3', 0.0, 0.0, 0.0, 2.0),
    ('y_neg', 0.0, -0.40, 0.0, 0.75),
    ('pause4', 0.0, 0.0, 0.0, 2.0),
    ('wz_pos', 0.0, 0.0, 0.80, 1.0),
    ('pause5', 0.0, 0.0, 0.0, 2.0),
    ('wz_neg', 0.0, 0.0, -0.80, 1.0),
    ('static_b', 0.0, 0.0, 0.0, 3.0),
]


class OdomCalibrationMotion(Node):
    def __init__(self):
        super().__init__('odom_calibration_motion')
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def send(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def run(self):
        period = 1.0 / PUBLISH_RATE_HZ
        total = sum(seg[4] for seg in SEGMENTS)
        self.get_logger().info(f'序列共 {len(SEGMENTS)} 段，预计 {total:.1f} s')
        start_wall = time.time()

        for name, vx, vy, wz, duration in SEGMENTS:
            self.get_logger().info(
                f'SEG {name} begin t={time.time() - start_wall:.2f} '
                f'cmd=({vx}, {vy}, {wz}) dur={duration}')
            seg_end = time.monotonic() + duration
            while time.monotonic() < seg_end:
                self.send(vx, vy, wz)
                time.sleep(period)
            self.send(0.0, 0.0, 0.0)
            self.get_logger().info(
                f'SEG {name} end t={time.time() - start_wall:.2f}')

        self.send(0.0, 0.0, 0.0)
        self.get_logger().info('序列结束，已发零速')


def main():
    rclpy.init()
    node = OdomCalibrationMotion()
    try:
        print('车体应着地、周围空旷或已确认安全，急停旁有人。')
        print('按回车开始，Ctrl-C 中止：', end='', flush=True)
        sys.stdin.readline()
        node.run()
    except KeyboardInterrupt:
        node.send(0.0, 0.0, 0.0)
        print('\n已中止，零速已下发')
    finally:
        for _ in range(5):
            node.send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
