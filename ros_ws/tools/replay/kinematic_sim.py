#!/usr/bin/env python3
"""Bag-replay odometry source: integrates /cmd_vel into odom->base_footprint.

Replaces Point-LIO for offline Nav2 tuning. The actuator model lives in a JSON
file so the same node can run "measured" (gain + deadband + lag) or "ideal"
(everything 1/0) to separate parameter effects from chassis defects.

The deadband is applied in the chassis frame, not the ROS frame: cmd_vel_to_unified
rotates (vx, vy) by frame_yaw before the firmware converts to RC counts, and
rc_deadband_limit() runs on those post-rotation channels. So a pure +x command is
zeroed below deadband/cos(frame_yaw) -- ~0.199 m/s for the current -45 deg.
"""

import json
import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

DEFAULT_PLANT = {
    "v_gain": 1.0,
    "wz_gain": 1.0,
    "deadband_v": 0.0,
    "deadband_w": 0.0,
    "deadband_frame": "chassis",
    "frame_yaw_deg": -45.0,
    "lag_tau": 0.0,
    "latency": 0.0,
}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class KinematicSim(Node):
    def __init__(self):
        super().__init__("kinematic_sim")

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odom_topic", "odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_child_frame", "gimbal_yaw")
        self.declare_parameter("init_x", -0.03)
        self.declare_parameter("init_y", 0.005)
        self.declare_parameter("init_yaw", 0.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("plant_json", "")

        gp = self.get_parameter
        self.cmd_topic = gp("cmd_vel_topic").value
        self.odom_topic = gp("odom_topic").value
        self.odom_frame = gp("odom_frame").value
        self.base_frame = gp("base_frame").value
        self.odom_child = gp("odom_child_frame").value
        self.rate = float(gp("publish_rate").value)
        self.cmd_timeout = float(gp("cmd_timeout").value)

        plant = dict(DEFAULT_PLANT)
        path = gp("plant_json").value
        if path:
            with open(path) as f:
                plant.update({k: v for k, v in json.load(f).items() if not k.startswith("_")})
        self.plant = plant
        self.frame_yaw = math.radians(float(plant.get("frame_yaw_deg", -45.0)))
        self.dt = 1.0 / self.rate

        self.x = float(gp("init_x").value)
        self.y = float(gp("init_y").value)
        self.yaw = float(gp("init_yaw").value)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.cmds = deque(maxlen=200)
        self.last_cmd_time = None
        self._last_t = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 10)
        self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f"kinematic_sim: plant={json.dumps({k: plant[k] for k in sorted(plant)})} "
            f"init=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) rate={self.rate} Hz"
        )

    # ---------------------------------------------------------------- plant

    def cmd_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.cmds.append((now, msg.linear.x, msg.linear.y, msg.angular.z))
        self.last_cmd_time = now

    def _pick_cmd(self, now):
        latency = float(self.plant.get("latency", 0.0))
        cutoff = now - latency
        chosen = (0.0, 0.0, 0.0)
        for stamp, vx, vy, wz in self.cmds:
            if stamp <= cutoff:
                chosen = (vx, vy, wz)
            else:
                break
        if self.last_cmd_time is None or now - self.last_cmd_time > self.cmd_timeout:
            return 0.0, 0.0, 0.0
        return chosen

    def _apply_deadband(self, vx, vy):
        dv = float(self.plant.get("deadband_v", 0.0))
        if self.plant.get("deadband_frame", "chassis") != "chassis" or dv <= 0.0:
            return (0.0 if abs(vx) < dv else vx), (0.0 if abs(vy) < dv else vy)
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        cx, cy = c * vx - s * vy, s * vx + c * vy
        if abs(cx) < dv:
            cx = 0.0
        if abs(cy) < dv:
            cy = 0.0
        return c * cx + s * cy, -s * cx + c * cy

    def _actuate(self, vx, vy, wz, dt):
        vx, vy = self._apply_deadband(vx, vy)
        vx *= float(self.plant.get("v_gain", 1.0))
        vy *= float(self.plant.get("v_gain", 1.0))
        dw = float(self.plant.get("deadband_w", 0.0))
        if abs(wz) < dw:
            wz = 0.0
        wz *= float(self.plant.get("wz_gain", 1.0))
        tau = float(self.plant.get("lag_tau", 0.0))
        if tau > 0.0:
            a = min(1.0, dt / tau)
            return self.vx + (vx - self.vx) * a, self.vy + (vy - self.vy) * a, self.wz + (wz - self.wz) * a
        return vx, vy, wz

    # ---------------------------------------------------------------- step

    def step(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        # Integrate over the time that actually elapsed, not the nominal timer
        # period. Under --clock the timer fires late (40 Hz for a 50 Hz timer),
        # and using self.dt made the simulated car travel 20% less distance
        # than its own /odometry twist reported.
        dt = self.dt if self._last_t is None else min(max(t - self._last_t, 0.0), 0.2)
        self._last_t = t
        vx_cmd, vy_cmd, wz_cmd = self._pick_cmd(t)
        self.vx, self.vy, self.wz = self._actuate(vx_cmd, vy_cmd, wz_cmd, dt)

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        wx = c * self.vx - s * self.vy
        wy = s * self.vx + c * self.vy
        self.x += wx * dt
        self.y += wy * dt
        self.yaw += self.wz * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.odom_child
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
