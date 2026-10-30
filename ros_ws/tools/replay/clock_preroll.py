#!/usr/bin/env python3
"""Publish /clock ahead of a bag replay so Nav2 can finish autostart first.

Every Nav2 node in this stack runs with use_sim_time:=true. Their lifecycle
timers only fire when /clock advances, so if bag playback were the first clock
source, the stack would still be activating during the first seconds of recorded
world -- and those seconds would be silently dropped.

This node advances /clock (fast, e.g. 5x) from `start_sec` to `end_sec`, which
must both lie *before* the bag's first message stamp. Bag playback then takes
over monotonically at its own start time.

It does NOT exit at end_sec: it holds /clock there until it is killed. Exiting
once made lifecycle activation stall whenever configuration took longer than the
preroll -- sim time froze, the remaining nodes never came up and the run died
with "lifecycle never active". The caller (run_variant.py) kills it as soon as
every node is active, then starts playback.
"""

import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockPreroll(Node):
    def __init__(self):
        super().__init__("clock_preroll")
        self.declare_parameter("start_sec", 0.0)
        self.declare_parameter("end_sec", 0.0)
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("tick_hz", 50.0)

        self.start = float(self.get_parameter("start_sec").value)
        self.end = float(self.get_parameter("end_sec").value)
        self.rate = float(self.get_parameter("rate").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)

        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.wall0 = time.monotonic()
        self.finished = False
        self._announced = False
        self.create_timer(1.0 / self.tick_hz, self.tick)
        self.get_logger().info(
            f"preroll {self.start:.3f} -> {self.end:.3f} at {self.rate}x")

    def tick(self):
        elapsed = time.monotonic() - self.wall0
        sim = self.start + elapsed * self.rate
        if sim >= self.end:
            sim = self.end
            self.finished = True
        msg = Clock()
        sec = int(sim)
        msg.clock.sec = sec
        msg.clock.nanosec = int((sim - sec) * 1e9)
        self.pub.publish(msg)
        if self.finished and not self._announced:
            self._announced = True
            self.get_logger().info(
                f"preroll reached {sim:.3f}; holding until killed")


def main():
    rclpy.init()
    node = ClockPreroll()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
