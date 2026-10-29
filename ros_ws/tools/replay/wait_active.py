#!/usr/bin/env python3
"""Block until every listed lifecycle node reports ACTIVE.

    python3 wait_active.py --ros-args -p nodes:=map_server,bt_navigator -p timeout:=90

Prints ALL_ACTIVE and exits 0, or MISSING <names> and exits 1.

One process, one persistent client per node: `ros2 lifecycle get` per node per
second spawns eight DDS participants a second, and under a busy costmap its 1 s
service deadline expires, so the CLI reports "not active" for nodes that are in
fact active (observed as `failed to send response ... (timeout)` in the servers).
"""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


class WaitActive(Node):
    def __init__(self):
        super().__init__("wait_active")
        self.declare_parameter("nodes", "")
        self.declare_parameter("timeout", 90.0)
        self.declare_parameter("poll_period", 0.5)

        names = [n.strip().lstrip("/") for n in
                 str(self.get_parameter("nodes").value).split(",") if n.strip()]
        self.state_clients = {n: self.create_client(GetState, f"/{n}/get_state") for n in names}
        self.timeout = float(self.get_parameter("timeout").value)
        self.period = float(self.get_parameter("poll_period").value)

    def poll(self):
        missing, states = [], {}
        for name, client in self.state_clients.items():
            if not client.service_is_ready():
                missing.append(name)
                states[name] = "no-service"
                continue
            fut = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            res = fut.result() if fut.done() else None
            if res is None:
                missing.append(name)
                states[name] = "no-response"
            else:
                states[name] = res.current_state.label
                if res.current_state.id != State.PRIMARY_STATE_ACTIVE:
                    missing.append(name)
        return missing, states

    def run(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline and rclpy.ok():
            missing, states = self.poll()
            if not missing:
                print("ALL_ACTIVE " + " ".join(
                    f"{k}={v}" for k, v in states.items()), flush=True)
                return 0
            time.sleep(self.period)
        print("MISSING " + " ".join(
            f"{k}={states.get(k, '?')}" for k in missing), flush=True)
        return 1


def main():
    rclpy.init()
    node = WaitActive()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
