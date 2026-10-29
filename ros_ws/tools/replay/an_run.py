#!/usr/bin/env python3
"""Score one real-robot navigation run against the tuning baselines.

Prints the same metric table for every run so successive parameter changes can
be compared directly.  Works on a bag that contains /cmd_vel, /unified_cmd,
/odometry, /chassis/state, /plan and (for the map-frame lateral error) /tf.

Usage:
    python3 an_run.py <bag_dir> [--json out.json]

Metrics
    route_s          total bag span
    dead_cmd_s       time with a non-zero command but the wheels not turning
                     (the chassis deadband swallowing the command)
    hold_s           time with a zero command
    v_swing_frac     share of the cruise spent in the 0.3-0.6 m/s half-speed band
    gain             odometry speed / commanded chassis speed
    lat_p50/p90      distance from the planned path, in the map frame
    final_err        distance from the requested goal at the end, map frame
"""

import bisect
import json
import math
import sys

import numpy as np
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.serialization import deserialize_message
from sentry_interfaces.msg import ChassisState, UnifiedCmd
from tf2_msgs.msg import TFMessage

TOPICS = ["/cmd_vel", "/unified_cmd", "/odometry", "/chassis/state", "/plan", "/tf"]


def load(path):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    odom, plans, cmds, uni, chas, tf = [], [], [], [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        t = stamp * 1e-9
        if topic == "/odometry":
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            odom.append((t, p.x, p.y,
                         m.twist.twist.linear.x, m.twist.twist.linear.y, yaw))
        elif topic == "/plan":
            m = deserialize_message(data, Path)
            if m.poses:
                plans.append((t, [(q.pose.position.x, q.pose.position.y) for q in m.poses]))
        elif topic == "/cmd_vel":
            m = deserialize_message(data, Twist)
            cmds.append((t, m.linear.x, m.linear.y, m.angular.z))
        elif topic == "/unified_cmd":
            m = deserialize_message(data, UnifiedCmd)
            uni.append((t, m.vx_mm_s / 1000.0, m.vy_mm_s / 1000.0))
        elif topic == "/chassis/state":
            m = deserialize_message(data, ChassisState)
            chas.append((t, max(abs(v) for v in m.speed_mps)))
        elif topic == "/tf":
            m = deserialize_message(data, TFMessage)
            for tr in m.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    q = tr.transform.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    tf.append((t, tr.transform.translation.x,
                               tr.transform.translation.y, yaw))
    for lst in (odom, plans, cmds, uni, chas, tf):
        lst.sort(key=lambda x: x[0])
    return odom, plans, cmds, uni, chas, tf


def nearest(seq, t):
    if not seq:
        return None
    i = bisect.bisect_left([s[0] for s in seq], t)
    i = min(i, len(seq) - 1)
    if i > 0 and abs(seq[i - 1][0] - t) < abs(seq[i][0] - t):
        i -= 1
    return seq[i]


def episodes(times, flags, min_dur):
    """(start, duration) of runs of True lasting at least min_dur."""
    out, start, last = [], None, None
    for i in range(1, len(times)):
        if flags[i]:
            if start is None:
                start = times[i - 1]
            last = times[i]
        elif start is not None:
            if last - start >= min_dur:
                out.append((start, last - start))
            start = None
    if start is not None and last - start >= min_dur:
        out.append((start, last - start))
    return out


def to_map(sample, x, y):
    _, tx, ty, yaw = sample
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def analyze(path):
    odom, plans, cmds, uni, chas, tf = load(path)
    if not odom or not cmds:
        raise SystemExit(f"{path}: need /odometry and /cmd_vel")

    t0 = min(odom[0][0], cmds[0][0])
    t_end = max(odom[-1][0], cmds[-1][0])
    out = {"bag": path, "route_s": round(t_end - t0, 1)}

    # --- command vs wheels -------------------------------------------------
    ct = [c[0] for c in cmds]
    cv = [math.hypot(c[1], c[2]) for c in cmds]
    if odom:
        # "commanded but not moving": the pose barely changes across a 0.5 s
        # window centred on the sample.  Catches both the translation deadband
        # and a rotation command too small to overcome the wz deadband -- a
        # wheel-speed test misses the latter because the wheels do turn.
        dead, hold = [], []
        for c, v in zip(cmds, cv):
            t = c[0]
            # A rotation-only command still counts as a command: a wz below the
            # chassis deadband leaves the robot sitting there just as a tiny
            # translation command does.
            active = v > 0.02 or abs(c[3]) > 0.02
            a, b = nearest(odom, t - 0.5), nearest(odom, t + 0.5)
            still = True
            if a and b:
                dyaw = abs(math.atan2(math.sin(b[5] - a[5]), math.cos(b[5] - a[5])))
                still = math.hypot(b[1] - a[1], b[2] - a[2]) < 0.05 and dyaw < 0.1
            dead.append(still and active)
            hold.append(not active)
        eps = episodes(ct, dead, 0.5)
        out["dead_cmd_s"] = round(sum(d for _, d in eps), 1)
        out["dead_episodes"] = len(eps)
        out["dead_at_s"] = [round(s - t0, 1) for s, _ in eps]
        out["hold_s"] = round(sum(d for _, d in episodes(ct, hold, 0.5)), 1)

    # --- cruise speed ------------------------------------------------------
    moving = [v for v in cv if v > 0.02]
    if moving:
        out["cmd_mean_ms"] = round(float(np.mean(moving)), 2)
        out["cmd_p95_ms"] = round(float(np.percentile(moving, 95)), 2)
        out["v_swing_frac"] = round(float(np.mean([0.3 <= v <= 0.6 for v in moving])), 2)
    ot = [o[0] for o in odom]
    ov = [math.hypot(o[3], o[4]) for o in odom]
    out["odom_mean_ms"] = round(float(np.mean([v for v in ov if v > 0.05])), 2)

    if uni:
        gains = []
        for t, vx, vy in uni:
            v = math.hypot(vx, vy)
            if v < 0.25:
                continue
            s = nearest(odom, t + 0.15)
            if s:
                gains.append(math.hypot(s[3], s[4]) / v)
        if gains:
            out["gain"] = round(float(np.median(gains)), 2)

    # --- path tracking, map frame -----------------------------------------
    if tf and plans:
        lat, base = [], []
        for o in odom:
            m = nearest(cmds, o[0])
            if m is None or math.hypot(m[1], m[2]) <= 0.02:
                continue
            tf_s = nearest(tf, o[0])
            if tf_s is None:
                continue
            mx, my = to_map(tf_s, o[1], o[2])
            p = nearest(plans, o[0])[1]
            d = min(math.hypot(px - mx, py - my) for px, py in p)
            lat.append(d)
        if lat:
            out["lat_p50_m"] = round(float(np.percentile(lat, 50)), 3)
            out["lat_p90_m"] = round(float(np.percentile(lat, 90)), 3)
            out["lat_max_m"] = round(float(max(lat)), 2)
        goal = plans[-1][1][-1]
        last = odom[-1]
        tf_s = nearest(tf, last[0])
        if tf_s:
            mx, my = to_map(tf_s, last[1], last[2])
            out["final_err_m"] = round(math.hypot(goal[0] - mx, goal[1] - my), 3)
            out["goal"] = [round(goal[0], 2), round(goal[1], 2)]

    if plans:
        dur = plans[-1][0] - plans[0][0]
        out["replan_hz"] = round(len(plans) / dur, 2) if dur > 0 else 0.0
    return out


def main():
    res = analyze(sys.argv[1])
    if "--json" in sys.argv:
        with open(sys.argv[sys.argv.index("--json") + 1], "w") as f:
            json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
