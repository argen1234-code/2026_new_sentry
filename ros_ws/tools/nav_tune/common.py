#!/usr/bin/env python3
"""Shared readers and helpers for the nav2 tuning tools.

Every tool in this directory works on the *same* inputs -- a rosbag2 directory,
either a real-robot recording or a replay-bench recording -- so the numbers are
comparable across the two. Nothing here touches a running robot.
"""

import math
import os

import numpy as np
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

ODOM = "/odometry"
PLAN = "/plan"
CMD = "/cmd_vel"
CMD_CONTROLLER = "/cmd_vel_controller"
GLOBAL_COSTMAP = "/global_costmap/costmap"
LOCAL_COSTMAP = "/local_costmap/costmap"
TF = "/tf"
TF_STATIC = "/tf_static"

# Costmap cells at or above this are treated as lethal. Note 254 is the raw
# value; what costmap_2d *publishes* is compressed to 0-100, so 100 -- not 254 --
# is the lethal value on the wire
# (`nav2_costmap_2d/costmap_2d_publisher.hpp:175`).
LETHAL_PUBLISHED = 99


def reader(path, topics=None):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return r


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def grid_of(msg):
    return np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)


def cell_of(msg, x, y):
    """Map/world (x, y) -> (row, col) in a costmap message's grid."""
    info = msg.info
    res = info.resolution
    col = int((x - info.origin.position.x) / res)
    row = int((y - info.origin.position.y) / res)
    if 0 <= row < info.height and 0 <= col < info.width:
        return row, col
    return None


def max_cost_near(msg, x, y, radius):
    """Largest published cost within `radius` (m) of (x, y); None if off-grid."""
    rc = cell_of(msg, x, y)
    if rc is None:
        return None
    r, c = rc
    k = max(1, int(round(radius / msg.info.resolution)))
    g = grid_of(msg)
    win = g[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
    return int(win.max()) if win.size else None


def compose(trans, odom_pose):
    """map->odom transform tuple (dx, dy, dyaw) applied to an odom pose."""
    dx, dy, dyaw = trans
    x, y, yaw = odom_pose
    c, s = math.cos(dyaw), math.sin(dyaw)
    return (dx + c * x - s * y, dy + s * x + c * y, yaw + dyaw)


def load(path, topics=None, want=(ODOM, PLAN, CMD, CMD_CONTROLLER,
                                   GLOBAL_COSTMAP, LOCAL_COSTMAP, TF, TF_STATIC)):
    """Read a bag into plain lists, with message counts for every topic seen.

    Returns a dict:
      counts  -- {topic: (n, t_first, t_last)}
      odom    -- [(t, x, y, yaw, vx, vy, wz)]
      plan    -- [(t, [(x, y), ...])]
      cmd     -- [(t, vx, vy, wz)]            (cmd_vel)
      cmd_ctrl-- [(t, vx, vy, wz)]            (cmd_vel_controller, may be empty)
      gcost   -- [(t, OccupancyGrid)]
      lcost   -- [(t, OccupancyGrid)]
      map2odom-- [(t, x, y, z, yaw)]          (from /tf and /tf_static)
      frames  -- (frame_ids of the two costmaps, e.g. ('map', 'odom'))
    """
    counts, odom, plans, cmds, cmd_ctrl, gcost, lcost, m2o = {}, [], [], [], [], [], [], []
    gframe = lframe = None
    r = reader(path, topics if topics else want)
    while r.has_next():
        topic, data, ts = r.read_next()
        t = ts * 1e-9
        c = counts.get(topic)
        if c is None:
            counts[topic] = [1, t, t]
        else:
            c[0] += 1
            c[2] = t
        if topic == ODOM:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            tw = m.twist.twist
            odom.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation),
                         tw.linear.x, tw.linear.y, tw.angular.z))
        elif topic == PLAN:
            m = deserialize_message(data, Path)
            if m.poses:
                plans.append((t, [(q.pose.position.x, q.pose.position.y) for q in m.poses]))
        elif topic == CMD:
            m = deserialize_message(data, Twist)
            cmds.append((t, m.linear.x, m.linear.y, m.angular.z))
        elif topic == CMD_CONTROLLER:
            m = deserialize_message(data, Twist)
            cmd_ctrl.append((t, m.linear.x, m.linear.y, m.angular.z))
        elif topic == GLOBAL_COSTMAP:
            m = deserialize_message(data, OccupancyGrid)
            gframe = gframe or m.header.frame_id
            gcost.append((t, m))
        elif topic == LOCAL_COSTMAP:
            m = deserialize_message(data, OccupancyGrid)
            lframe = lframe or m.header.frame_id
            lcost.append((t, m))
        elif topic in (TF, TF_STATIC):
            m = deserialize_message(data, TFMessage)
            for tr in m.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    p, q = tr.transform.translation, tr.transform.rotation
                    m2o.append((t, p.x, p.y, p.z, yaw_of(q)))
    for lst in (odom, plans, cmds, cmd_ctrl, gcost, lcost, m2o):
        lst.sort(key=lambda x: x[0])
    return {"counts": counts, "odom": odom, "plan": plans, "cmd": cmds,
            "cmd_ctrl": cmd_ctrl, "gcost": gcost, "lcost": lcost,
            "map2odom": m2o, "gframe": gframe, "lframe": lframe}


def pose_at(series, t):
    """Binary-search the last sample at or before t (series sorted by stamp)."""
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return series[lo]


def nearest(series, t):
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(series[lo - 1][0] - t) < abs(series[lo][0] - t):
        return series[lo - 1]
    return series[lo]


def map_pose(data, t):
    """Robot pose in the costmap frames, from /odometry + the last map->odom."""
    o = pose_at(data["odom"], t)
    if data["map2odom"]:
        m = pose_at(data["map2odom"], t)
        return compose((m[1], m[2], m[4]), (o[1], o[2], o[3]))
    return (o[1], o[2], o[3])


def table(rows, header):
    """Render an aligned text table; rows is a list of tuples of str."""
    widths = [max(len(header[i]), *(len(r[i]) for r in rows)) for i in range(len(header))]
    out = ["  ".join(h.ljust(w) for h, w in zip(header, widths)),
           "  ".join("-" * w for w in widths)]
    out += ["  ".join(c.ljust(w) for c, w in zip(r, widths)) for r in rows]
    return "\n".join(out)


def load_map(yaml_path):
    """Read a nav2 map yaml + its PGM. Returns (grid, resolution, ox, oy).

    grid is indexed like the costmap publisher: row 0 is the *bottom* (lowest y),
    so `grid[row, col]` matches (x = ox + col*res, y = oy + row*res).
    """
    import yaml
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    with open(image, "rb") as f:
        data = f.read()
    parts, idx = [], 0
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] != b"\n":
                idx += 1
            continue
        tok = b""
        while not data[idx:idx + 1].isspace():
            tok += data[idx:idx + 1]
            idx += 1
        parts.append(tok)
    w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(data[idx + 1:idx + 1 + w * h], dtype=np.uint8).reshape(h, w)
    grid = img[::-1, :]  # PGM rows run top-down; costmaps run bottom-up
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    return grid, float(meta["resolution"]), ox, oy


def free_run(grid, res, ox, oy, x, y, axis="y", free_thresh=250):
    """Free run through (x, y) along `axis`; None if that cell isn't free.

    Returns (width, lo, hi) in world units on that axis. A corridor running
    along x is measured with axis="y" -- that is its width. Measuring along x
    gives the corridor's *length*, which is the classic way to get a 5 m "width"
    out of a 2 m corridor.

    free_thresh defaults to 250 because nav2's trinary PGM uses 254=free,
    205=unknown, 0=occupied; anything below 250 is not "known free" and must not
    be counted as passable width.
    """
    row = int(round((y - oy) / res))
    col = int(round((x - ox) / res))
    if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
        return None
    if grid[row, col] < free_thresh:
        return None
    if axis == "x":
        lo, hi, line, origin, lim = col, col, grid[row], ox, grid.shape[1]
        step = lambda i: line[i]
    else:
        lo, hi, line, origin, lim = row, row, grid[:, col], oy, grid.shape[0]
        step = lambda i: line[i]
    while lo > 0 and step(lo - 1) >= free_thresh:
        lo -= 1
    while hi + 1 < lim and step(hi + 1) >= free_thresh:
        hi += 1
    return ((hi - lo + 1) * res, origin + lo * res, origin + (hi + 1) * res)


def rate(n, t_first, t_last):
    span = t_last - t_first
    return n / span if span > 0 else 0.0
