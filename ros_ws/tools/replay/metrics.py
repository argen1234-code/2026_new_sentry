#!/usr/bin/env python3
"""Score a recording against the phase-X baselines.

Works on both a real-robot bag and a replay bag, so the reference numbers are
produced by the same code that scores the variants (no apples-to-oranges).

Usage:
    python3 metrics.py <bag_dir> [--json out.json]
"""

import json
import math
import statistics
import sys

import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist

TOPICS = [
    "/odometry",
    "/plan",
    "/cmd_vel",
    "/global_costmap/costmap",
]


def reader(path, topics):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(path):
    r = reader(path, TOPICS)
    odom, plans, cmds, costs = [], [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        t = stamp * 1e-9
        if topic == "/odometry":
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            odom.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation),
                         m.twist.twist.linear.x, m.twist.twist.linear.y,
                         m.twist.twist.angular.z))
        elif topic == "/plan":
            m = deserialize_message(data, Path)
            if m.poses:
                plans.append((t, [(q.pose.position.x, q.pose.position.y) for q in m.poses]))
        elif topic == "/cmd_vel":
            m = deserialize_message(data, Twist)
            cmds.append((t, m.linear.x, m.linear.y, m.angular.z))
        elif topic == "/global_costmap/costmap":
            m = deserialize_message(data, OccupancyGrid)
            costs.append((t, m))
    for lst in (odom, plans, cmds, costs):
        lst.sort(key=lambda x: x[0])
    return odom, plans, cmds, costs


def pose_at(odom, t):
    if not odom:
        return 0.0, 0.0, 0.0
    lo, hi = 0, len(odom) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if odom[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    r = odom[lo]
    return r[1], r[2], r[3]


def to_robot_frame(points, pose):
    x0, y0, yaw = pose
    c, s = math.cos(-yaw), math.sin(-yaw)
    out = []
    for x, y in points:
        dx, dy = x - x0, y - y0
        out.append((c * dx - s * dy, s * dx + c * dy))
    return out


def world_velocity(odom, win=0.3):
    """Smoothed world-frame velocity from pose differences.

    The recorded /odometry twist is neither cleanly child-frame nor world-frame
    (median error 0.20 vs 0.17 m/s either way -- it is time-shifted against the
    pose), so direction is taken from the pose itself.
    """
    out = []
    j = 0
    for i, r in enumerate(odom):
        while j < i and r[0] - odom[j][0] > win:
            j += 1
        if j == i or r[0] - odom[j][0] < 1e-3:
            out.append((0.0, 0.0))
            continue
        dt = r[0] - odom[j][0]
        out.append(((r[1] - odom[j][1]) / dt, (r[2] - odom[j][2]) / dt))
    return out


def arclength(points):
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return np.array(s)


def lateral_profile(points, lo=0.5, hi=3.0, step=0.05):
    """Lateral offset y as a function of distance travelled along the path."""
    if len(points) < 3:
        return None, None
    s = arclength(points)
    if s[-1] < hi:
        return None, None
    p = np.array(points)
    xs = np.arange(lo, hi + 1e-9, step)
    ys = np.interp(xs, s, p[:, 1])
    return xs, ys


def jitter(odom, plans):
    """p90 lateral difference between consecutive plans, in a common robot frame.

    Both plans are expressed in the robot frame at the *later* plan's stamp, so
    the difference is path shape only, not robot motion between the two plans.
    """
    diffs = []
    pair_max = []
    prev_pts = None
    for t, pts in plans:
        pose = pose_at(odom, t)
        cur = to_robot_frame(pts, pose)
        if prev_pts is not None:
            prev = to_robot_frame(prev_pts, pose)
            xs, ya = lateral_profile(prev)
            if xs is not None:
                _, yb = lateral_profile(cur)
                if yb is not None:
                    d = np.abs(ya - yb)
                    diffs.extend(d.tolist())
                    pair_max.append(float(d.max()))
        prev_pts = pts
    if not diffs:
        return {}
    d = np.array(diffs)
    pm = np.array(pair_max)
    return {
        "jitter_p50_m": float(np.percentile(d, 50)),
        "jitter_p90_m": float(np.percentile(d, 90)),
        "jitter_max_m": float(d.max()),
        "pairs_over_015_frac": float((pm > 0.15).mean()),
        "n_plan_pairs": len(pair_max),
    }


def speed_stats(odom):
    if not odom:
        return {}
    sp = np.array([math.hypot(r[4], r[5]) for r in odom])
    moving = sp[sp > 0.02]
    return {
        "speed_moving_mean_ms": float(moving.mean()) if moving.size else 0.0,
        "speed_moving_median_ms": float(np.median(moving)) if moving.size else 0.0,
        "frac_speed_lt_005": float((sp < 0.05).mean()),
    }


def longest_stop(odom):
    best = 0.0
    run = 0.0
    for a, b in zip(odom, odom[1:]):
        dt = b[0] - a[0]
        if math.hypot(b[4], b[5]) < 0.05:
            run += dt
            best = max(best, run)
        else:
            run = 0.0
    return best


def stop_count(odom, min_speed=0.05, min_dur=0.5):
    n = 0
    run = 0.0
    in_stop = False
    for a, b in zip(odom, odom[1:]):
        if math.hypot(b[4], b[5]) < min_speed:
            run += b[0] - a[0]
            if run >= min_dur and not in_stop:
                n += 1
                in_stop = True
        else:
            run = 0.0
            in_stop = False
    return n


def cmd_accel(cmds):
    if len(cmds) < 3:
        return 0.0
    accs = []
    for a, b in zip(cmds, cmds[1:]):
        dt = b[0] - a[0]
        if dt <= 1e-4 or dt > 0.5:
            continue
        ax = (b[1] - a[1]) / dt
        ay = (b[2] - a[2]) / dt
        accs.append(math.hypot(ax, ay))
    if not accs:
        return 0.0
    return float(np.percentile(accs, 95))


def lethal_points(msg, max_points=2000):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    idx = np.argwhere(data >= 99)
    if idx.size == 0:
        return None
    if len(idx) > max_points:
        sel = np.random.default_rng(0).choice(len(idx), max_points, replace=False)
        idx = idx[sel]
    res = msg.info.resolution
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    return np.column_stack([ox + (idx[:, 1] + 0.5) * res, oy + (idx[:, 0] + 0.5) * res])


def plan_clearance(plans, costs):
    """Minimum plan-to-lethal distance over the run (global costmap, map frame)."""
    if not plans or not costs:
        return None
    vals = []
    for t, msg in costs:
        lethal = lethal_points(msg)
        if lethal is None:
            continue
        near = min(plans, key=lambda p: abs(p[0] - t))
        p = np.array(near[1])
        if len(p) == 0:
            continue
        d = np.hypot(p[:, None, 0] - lethal[None, :, 0], p[:, None, 1] - lethal[None, :, 1])
        vals.append(float(d.min()))
    if not vals:
        return None
    return {"min_clearance_m": float(np.min(vals)), "median_clearance_m": float(np.median(vals))}


def goal_change_reaction(odom, plans):
    """Seconds between the goal switching and the robot actually reversing."""
    if len(plans) < 2:
        return None
    base = plans[0][1][-1]
    switch_t = None
    new_goal = None
    for t, pts in plans:
        g = pts[-1]
        if math.hypot(g[0] - base[0], g[1] - base[1]) > 1.0:
            switch_t = t
            new_goal = g
            break
    if switch_t is None:
        return None
    x0, y0, _ = pose_at(odom, switch_t)
    ux, uy = new_goal[0] - x0, new_goal[1] - y0
    nrm = math.hypot(ux, uy) or 1.0
    ux, uy = ux / nrm, uy / nrm
    # First moment the robot sustains motion toward the NEW goal for 0.5 s.
    vel = world_velocity(odom)
    run = 0.0
    run_start = None
    for r, (wx, wy) in zip(odom, vel):
        t = r[0]
        if t < switch_t:
            continue
        proj = wx * ux + wy * uy
        # 0.12 m/s, not 0.05: while the robot sits still and spins, pose jitter
        # alone produces +/-0.05-0.10 m/s, which would report a bogus fast reaction.
        if proj > 0.12:
            if run_start is None:
                run_start = t
            run = t - run_start
            if run >= 1.0:
                return {"goal_switch_t": switch_t, "reaction_s": round(run_start - switch_t, 2),
                        "new_goal": [round(new_goal[0], 2), round(new_goal[1], 2)]}
        else:
            run_start = None
            run = 0.0
    return {"goal_switch_t": switch_t, "reaction_s": None,
            "new_goal": [round(new_goal[0], 2), round(new_goal[1], 2)]}


def analyze(path):
    odom, plans, cmds, costs = load(path)
    if not odom:
        raise SystemExit(f"no /odometry in {path}")
    t0 = odom[0][0]
    dur = odom[-1][0] - t0
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(odom, odom[1:]))
    out = {
        "bag": path,
        "duration_s": round(dur, 2),
        "path_length_m": round(dist, 2),
        "n_odom": len(odom),
        "n_plans": len(plans),
        "replan_hz": round(len(plans) / dur, 2) if dur > 0 else 0.0,
        "final_pose": [round(odom[-1][1], 2), round(odom[-1][2], 2), round(odom[-1][3], 2)],
        "max_x": round(max(r[1] for r in odom), 2),
        "longest_stop_s": round(longest_stop(odom), 2),
        "stops_ge_05s": stop_count(odom),
        "cmd_accel_p95_ms2": round(cmd_accel(cmds), 2),
    }
    out.update(speed_stats(odom))
    out.update(jitter(odom, plans))
    pc = plan_clearance(plans, costs)
    if pc:
        out.update({k: round(v, 2) for k, v in pc.items()})
    gr = goal_change_reaction(odom, plans)
    if gr:
        out["goal_reaction"] = gr
    return out


def main():
    path = sys.argv[1]
    res = analyze(path)
    if "--json" in sys.argv:
        with open(sys.argv[sys.argv.index("--json") + 1], "w") as f:
            json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
