#!/usr/bin/env python3
"""Compare / change nav2 parameters on a *running* stack.

`ros2 param` already does get/set/dump; what it does not do is tell you whether
what a node actually holds matches the file you edited, or which keys in that
file no node ever declared (silently ignored -- nav2 does not complain).

    python3 params.py --diff                      # 跑着的栈 vs 仓库 yaml
    python3 params.py --diff --config other.yaml
    python3 params.py --dump /tmp/live_params     # 每个节点一份 live yaml
    python3 params.py --set controller_server.FollowPath.v_linear_max=0.6
    python3 params.py --get controller_server.FollowPath.v_linear_max

Read-only unless --set is used. This tool never enables the chassis and never
publishes anything.
"""

import argparse
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_CFG = os.path.join(
    REPO, "ros_ws", "src", "sentry_navigation", "pb2025_nav_bringup",
    "config", "reality", "nav2_params.yaml")

# yaml key -> running node. The costmaps are nested one level deeper in the file.
NODES = ["controller_server", "planner_server", "bt_navigator", "smoother_server",
         "behavior_server", "waypoint_follower", "velocity_smoother",
         "local_costmap.local_costmap", "global_costmap.global_costmap",
         "map_server", "slam_toolbox", "pointcloud_to_laserscan",
         "terrain_analysis", "terrain_analysis_ext"]


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def live_dump(node):
    """Effective parameters of one running node, or None if it is not up."""
    r = run(["ros2", "param", "dump", "/" + node.split(".")[0]])
    if r.returncode != 0 or not r.stdout.strip():
        return None
    return yaml.safe_load(r.stdout)


def file_params(cfg, node):
    d = cfg
    for part in node.split("."):
        if not isinstance(d, dict) or part not in d:
            return None
        d = d[part]
    return d.get("ros__parameters")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=DEFAULT_CFG)
    ap.add_argument("--diff", action="store_true", help="跑着的栈 vs 文件")
    ap.add_argument("--dump", default=None, help="把每个节点的 live 参数写到这个目录")
    ap.add_argument("--get", default=None, help="节点名.参数路径")
    ap.add_argument("--set", dest="assign", default=None, help="节点名.参数路径=值")
    ap.add_argument("--quiet", action="store_true", help="只打印不一致项")
    args = ap.parse_args()

    if args.get:
        node, _, keypath = args.get.partition(".")
        r = run(["ros2", "param", "get", "/" + node, keypath])
        sys.stdout.write(r.stdout or r.stderr)
        return 0 if r.returncode == 0 else 1

    if args.assign:
        lhs, _, value = args.assign.partition("=")
        node, _, keypath = lhs.partition(".")
        # Types must match the parameter's declared type; nav2 declares most as
        # double/bool/integer, and passing the yaml's spelling through keeps
        # "0.6" a double and "true" a bool.
        r = run(["ros2", "param", "set", "/" + node, keypath, value])
        sys.stdout.write(r.stdout or r.stderr)
        if r.returncode != 0:
            return 1
        back = run(["ros2", "param", "get", "/" + node, keypath])
        print(f"回读: {back.stdout.strip() or back.stderr.strip()}")
        return 0

    cfg = yaml.safe_load(open(args.config))
    if args.dump:
        os.makedirs(args.dump, exist_ok=True)
        for node in NODES:
            live = live_dump(node)
            if live is None:
                continue
            name = node.replace(".", "_") + ".yaml"
            with open(os.path.join(args.dump, name), "w") as f:
                yaml.safe_dump(live, f, allow_unicode=True)
            print(f"wrote {name}")

    if args.diff:
        n_diff = n_undeclared = 0
        for node in NODES:
            fp = file_params(cfg, node)
            if fp is None:
                continue
            live = live_dump(node)
            if live is None:
                print(f"[--] {node}: 没在跑（或参数服务没起来）")
                continue
            lp = live.get("/" + node.split(".")[0], {}).get("ros__parameters", {})
            for key, want in fp.items():
                if key == "use_sim_time" and lp.get(key) != want:
                    pass
                if key not in lp:
                    n_undeclared += 1
                    print(f"[未声明] {node}.{key}={want} —— 节点里没有这个参数，写了也不生效")
                    continue
                got = lp[key]
                if got != want:
                    n_diff += 1
                    print(f"[不一致] {node}.{key}: 文件 {want!r} ≠ 运行 {got!r}")
        print(f"\n不一致 {n_diff} 项，未声明 {n_undeclared} 项")
        return 1 if (n_diff or n_undeclared) else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
