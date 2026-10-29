#!/usr/bin/env python3
"""Send Nav2 goals on a simulated-time schedule, preempting at the right moment.

Goals are timed on the *bag* clock, not the robot position: the replayed world
(the person walking past) advances on bag time, so the goal change has to happen
at the same bag time as it did in the recording or the encounter changes shape.

goals_json example:
    [{"t": 0.0, "x": 11.97, "y": -0.97, "yaw": 0.0},
     {"t": 74.2, "x": 0.18, "y": -0.13, "yaw": 0.0}]
"""

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw):
    import math
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class GoalSchedule(Node):
    def __init__(self):
        super().__init__("goal_schedule")
        self.declare_parameter("goals_file", "")
        self.declare_parameter("t_origin", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("start_delay", 3.0)
        self.declare_parameter("result_timeout", 120.0)

        # Read from a file, not a -p value: rcl parses -p values as YAML, which
        # would turn a JSON goal list into a parameter list and break json.loads.
        with open(self.get_parameter("goals_file").value) as f:
            self.goals = json.load(f)
        self.t_origin = float(self.get_parameter("t_origin").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.timeout = float(self.get_parameter("result_timeout").value)

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.sent = 0
        # A preempted goal still delivers a (CANCELED) result; counting only
        # "sent" made the node quit the moment goal 2 was sent, cutting the run
        # in half. Wait for every goal to report back instead.
        self.pending = 0
        self.last_handle = None
        self.last_sent_at = None
        self.done = False
        self.t0 = None

        self.get_logger().info(f"waiting for navigate_to_pose; {len(self.goals)} goal(s)")
        self.client.wait_for_server()
        first = self.goals[0]["t"] + self.t_origin if self.goals else 0.0
        self.get_logger().info(f"server ready; first goal at sim t={first:.1f}")
        self.create_timer(0.05, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.sent >= len(self.goals):
            return
        goal = self.goals[self.sent]
        if self.now_s() >= self.t_origin + goal["t"]:
            self.send(goal)

    def send(self, goal):
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.frame_id
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal["x"])
        msg.pose.pose.position.y = float(goal["y"])
        qx, qy, qz, qw = yaw_to_quat(float(goal.get("yaw", 0.0)))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.sent += 1
        self.pending += 1
        self.last_sent_at = self.now_s()
        self.get_logger().info(
            f"[goal {self.sent}/{len(self.goals)}] t={self.last_sent_at - self.t_origin:.1f} "
            f"({goal['x']:.2f}, {goal['y']:.2f})")
        fut = self.client.send_goal_async(msg)
        fut.add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("goal rejected")
            return
        self.last_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        status = fut.result().status
        self.pending = max(0, self.pending - 1)
        self.get_logger().info(
            f"[goal result] status={status} "
            f"t={self.now_s() - self.t_origin:.1f} pending={self.pending}")
        if self.sent >= len(self.goals) and self.pending == 0:
            self.done = True


def main():
    import time

    rclpy.init()
    node = GoalSchedule()
    wall_deadline = time.monotonic() + node.timeout + 60.0
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Two independent guards: sim time catches a stuck goal while the
            # clock still runs, wall time catches a clock that froze at bag end.
            if node.last_sent_at is not None and node.now_s() - node.last_sent_at > node.timeout:
                node.get_logger().warn("result timeout (sim time), exiting with goals pending")
                break
            if time.monotonic() > wall_deadline:
                node.get_logger().warn("result timeout (wall time), exiting with goals pending")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
