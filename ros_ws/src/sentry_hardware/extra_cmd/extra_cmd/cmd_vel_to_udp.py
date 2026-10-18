import rclpy
from rclpy.node import Node
from sentry_decision.msg import CmdVelUdp
import socket
import json

class CmdVelToUdp(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_udp')
        
        # 声明参数
        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 19002)
        
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        
        # 创建UDP套接字
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # flag_wz 去抖参数：只有连续多次出现的新值才切换，过滤突发跳变
        self.flag_wz_switch_threshold = 3
        self.last_stable_flag_wz = None
        self.pending_flag_wz = None
        self.pending_flag_wz_count = 0
        
        # 订阅 /cmd_vel_udp（sentry_decision/CmdVelUdp 类型）
        self.subscription = self.create_subscription(
            CmdVelUdp,
            '/cmd_vel_udp',
            self.cmd_vel_udp_callback,
            10)
        
        self.get_logger().info(f'已启动 cmd_vel_to_udp 节点，目标: {self.udp_ip}:{self.udp_port}')

    def apply_deadzone(self, value):
        """
        死区补偿：
        - 值为 0 时，直接返回 0（到达目标后停止）
        - 绝对值在 1~24 之间（非零但太小，底盘可能不响应），补偿到 ±25
        - 绝对值 >= 25，原样返回
        """
        if value == 0:
            return 0
        abs_value = abs(value)
        if abs_value < 30:
            return 30 if value > 0 else -30
        return value

    def filter_flag_wz(self, flag_wz):
        """
        过滤 flag_wz 的瞬时突变：
        - 首次收到时直接采用
        - 与当前稳定值相同，直接通过并清空等待状态
        - 出现新值时，需要连续出现 `flag_wz_switch_threshold` 次才真正切换
        这样可以滤掉突然几个 2；如果一直是 2，则延迟几帧后切换到 2。
        """
        if self.last_stable_flag_wz is None:
            self.last_stable_flag_wz = flag_wz
            return flag_wz

        if flag_wz == self.last_stable_flag_wz:
            self.pending_flag_wz = None
            self.pending_flag_wz_count = 0
            return flag_wz

        if flag_wz != self.pending_flag_wz:
            self.pending_flag_wz = flag_wz
            self.pending_flag_wz_count = 1
            return self.last_stable_flag_wz

        self.pending_flag_wz_count += 1
        if self.pending_flag_wz_count >= self.flag_wz_switch_threshold:
            self.last_stable_flag_wz = flag_wz
            self.pending_flag_wz = None
            self.pending_flag_wz_count = 0
            self.get_logger().info(f'flag_wz 稳定切换为 {flag_wz}')

        return self.last_stable_flag_wz

    def send_udp_data(self, vx_mm_s, vy_mm_s, wz_mdeg_s, flag_wz):
        data = {
            "vx": vx_mm_s,
            "vy": vy_mm_s,
            "wz": wz_mdeg_s,
            "flag_wz": flag_wz
        }

        try:
            payload = json.dumps(data, separators=(",", ":")).encode('utf-8')
            self.sock.sendto(payload, (self.udp_ip, self.udp_port))
        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')

    def cmd_vel_udp_callback(self, msg):
        # 直接使用 CmdVelUdp 消息中的字段进行单位转换
        # vx/vy 单位为 m/s，转换为 mm/s（*130/*150 缩放系数）
        # wz 单位为 rad/s，转换为 mdeg/s（*350 缩放系数）
        
        vx_mm_s = int(msg.vx * 80)
        vy_mm_s = int(msg.vy * 80)
        wz_mdeg_s = int(msg.wz * 310)
        
        # 应用死区逻辑（vx 方向）
        vx_mm_s = self.apply_deadzone(vx_mm_s)

        # flag_wz 做去抖过滤，抑制突然几个 2 的突变
        raw_flag_wz = int(msg.flag_wz)
        flag_wz = self.filter_flag_wz(raw_flag_wz)

        self.send_udp_data(vx_mm_s, vy_mm_s, wz_mdeg_s, flag_wz)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUdp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
