#!/usr/bin/env python3
"""Narrow-passage report from one static bag.

Answers three questions on the same recording:
  1. how much of the gap survives in the local / global costmaps
  2. how much of what is left is phantom (lethal cell with no /map backing)
  3. does the planner actually get through, and how often the gap flickers

The robot is assumed static, so every frame shares one viewpoint.  The local
costmap is a rolling window centred on the robot, which makes it the cleanest
place to express "the corridor in front of the car".

Costmap values in the published OccupancyGrid are NOT the internal 0-255
cost; nav2 maps LETHAL_OBSTACLE (254) -> 100, INSCRIBED_INFLATED (253) -> 99,
NO_INFORMATION (255) -> -1.  ThetaStar's default lethal_cost is 253, so both
99 and 100 block the planner.

Usage: gap_report.py <bag_dir>
"""

import collections
import math
import sys

import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

LETHAL = 100
INSCRIBED = 99
UNKNOWN = -1

TOPICS = (
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/map",
    "/obstacle_scan",
    "/plan",
)


def open_reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return r


def load(path):
    """Stream the bag, keep only the small messages we analyse."""
    r = open_reader(path)
    out = {t: [] for t in TOPICS}
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic not in out:
            continue
        msg = deserialize_message(data, OccupancyGrid if topic != "/obstacle_scan"
                                 else LaserScan)
        if topic == "/plan":
            msg = deserialize_message(data, Path)
        out[topic].append((ts / 1e9, msg))
    return out


class Grid:
    """One OccupancyGrid with a cell lookup that tolerates out-of-range."""

    def __init__(self, msg):
        self.frame = msg.header.frame_id
        self.res = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.data = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)

    def cell(self, x, y):
        c = int(math.floor((x - self.ox) / self.res))
        rr = int(math.floor((y - self.oy) / self.res))
        if 0 <= rr < self.h and 0 <= c < self.w:
            return int(self.data[rr, c])
        return None

    def centre(self):
        return self.ox + self.w * self.res / 2.0, self.oy + self.h * self.res / 2.0

    def value(self):
        """Classify every cell: 0 free, 1 inflated, 2 lethal, 3 unknown."""
        v = self.data
        cls = np.zeros(v.shape, np.uint8)
        cls[(v > 0) & (v < INSCRIBED)] = 1
        cls[(v == INSCRIBED) | (v == LETHAL)] = 2
        cls[v == UNKNOWN] = 3
        return cls


def run_length(cls_row, free_first):
    """Longest continuous run of a class along one row, as (start_col, length)."""
    best = (0, 0)
    cur = 0
    for i, c in enumerate(cls_row):
        if c == free_first:
            cur += 1
            if cur > best[1]:
                best = (i - cur + 1, cur)
        else:
            cur = 0
    return best


def corridor_profile(grid, x0, x1, y, step=0.10):
    """Per-bin class histogram along +x from the robot, at lateral offset y."""
    bins = []
    x = x0
    while x < x1:
        counts = collections.Counter()
        for dx in np.arange(0, step, grid.res):
            v = grid.cell(x + dx, y)
            if v is None:
                counts["out"] += 1
            elif v == UNKNOWN:
                counts["unk"] += 1
            elif v == LETHAL:
                counts["let"] += 1
            elif v == INSCRIBED:
                counts["ins"] += 1
            elif v > 0:
                counts["inf"] += 1
            else:
                counts["free"] += 1
        bins.append((x, counts))
        x += step
    return bins


def first_block(grid, x0, x1, y, step=0.01):
    """Distance along +x at which the cell stops being plannable (>=99), or None."""
    x = x0
    while x <= x1:
        v = grid.cell(x, y)
        if v is None:
            return None, "off-grid"
        if v == LETHAL:
            return x, "lethal"
        if v == INSCRIBED:
            return x, "inscribed"
        x += step
    return None, "clear"


def free_width_at(grid, x, y0, y1, step=0.01):
    """Widest continuous plannable span across (y0,y1) at longitudinal x."""
    runs = []
    cur = 0
    start = None
    y = y0
    while y <= y1:
        v = grid.cell(x, y)
        ok = v is not None and v != LETHAL and v != INSCRIBED and v != UNKNOWN
        if ok:
            if start is None:
                start = y
            cur += 1
        else:
            if start is not None:
                runs.append((start, y - step, cur * step))
            start, cur = None, 0
        y += step
    if start is not None:
        runs.append((start, y1, cur * step))
    runs.sort(key=lambda r: -r[2])
    return runs


def components(cls_occupied, max_size):
    """Connected-component sizes (4-neighbour) of a boolean grid."""
    seen = np.zeros(cls_occupied.shape, bool)
    sizes = []
    h, w = cls_occupied.shape
    for r0 in range(h):
        for c0 in range(w):
            if not cls_occupied[r0, c0] or seen[r0, c0]:
                continue
            q = collections.deque([(r0, c0)])
            seen[r0, c0] = True
            n = 0
            while q:
                r, c = q.popleft()
                n += 1
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and cls_occupied[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        q.append((rr, cc))
            sizes.append(n)
    return sizes


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    bag = sys.argv[1]
    d = load(bag)
    for t in TOPICS:
        print("%-26s %d msgs" % (t, len(d[t])))
    print()

    # ---------------------------------------------------------------- local
    loc = [Grid(m) for _, m in d["/local_costmap/costmap"]]
    if not loc:
        print("no local costmap in bag")
        return 1
    g = loc[-1]
    rx, ry = g.centre()
    print("=== 1. 局部代价地图（odom，滚动窗口，车在中心 %.2f, %.2f）===" % (rx, ry))
    print("grid %dx%d @ %.3f m, 覆盖 x %.2f..%.2f" %
          (g.w, g.h, g.res, g.ox, g.ox + g.w * g.res))

    print("\n-- 沿 +x 的走廊分类（车中线 y=0，每 0.10 m）--")
    for x, counts in corridor_profile(g, 0.30, 2.50, 0.0):
        tot = max(1, sum(counts.values()))
        print("  x=%4.2f  free %3d  inf %3d  ins %3d  LETHAL %3d  unk %3d" %
              (x, counts["free"], counts["inf"], counts["ins"], counts["let"], counts["unk"]))

    for y in (0.0, 0.2, -0.2, 0.4, -0.4):
        dist, why = first_block(g, 0.30, 2.45, y)
        print("  首次阻挡 y=%+.1f: %s" % (y, "无（到 2.45 m 通）" if dist is None
                                         else "%.2f m (%s)" % (dist, why)))

    print("\n-- 横断面自由宽度（可规划 = 非 lethal/inscribed/unknown）--")
    for x in (1.40, 1.60, 1.80, 2.00, 2.20, 2.40):
        runs = free_width_at(g, x, -2.4, 2.4)
        top = ", ".join("y %.2f..%.2f (%.2f m)" % r for r in runs[:2])
        print("  x=%4.2f  %s" % (x, top or "全被阻挡"))

    # -------------------------------------------------------------- flicker
    print("\n=== 2. 时间抖动（%d 帧局部代价地图）===" % len(loc))
    step = min(len(loc), 60)
    box = cls_stack = []
    for gg in loc[-step:]:
        # 前向矩形 x in [0.3,2.4], |y| <= 1.2 relative to that frame's centre
        cx, cy = gg.centre()
        cols = int((2.4 - 0.3) / gg.res)
        rows = int(2.4 / gg.res)
        c0 = int((cx + 0.3 - gg.ox) / gg.res)
        r0 = int((cy - 1.2 - gg.oy) / gg.res)
        cls_stack.append(gg.value()[r0:r0 + rows, c0:c0 + cols])
    st = np.array(cls_stack)
    blocked = (st >= 2) & (st <= 2)          # lethal or inscribed
    ever = blocked.any(axis=0) & ~blocked.all(axis=0)
    print("  区域 %d 格（x 0.30-2.40, |y|<=1.2）" % blocked[0].size)
    print("  近 %d 帧中 阻碍率 每帧: 中位 %.1f%%  最小 %.1f%%  最大 %.1f%%" %
          (step, 100 * np.median(blocked.mean(axis=(1, 2))),
           100 * blocked.mean(axis=(1, 2)).min(), 100 * blocked.mean(axis=(1, 2)).max()))
    print("  类别发生翻转的格: %.1f%%" % (100.0 * ever.sum() / blocked[0].size))
    # 中线走廊逐帧是否连通
    ok = []
    for gg in loc[-step:]:
        dist, why = first_block(gg, 0.30, 2.45, 0.0)
        ok.append(dist is None)
    print("  中线到 2.45 m 连通的帧: %d/%d (%.0f%%)" % (sum(ok), len(ok), 100.0 * sum(ok) / len(ok)))

    # --------------------------------------------------------------- global
    print("\n=== 3. 全局代价地图（map）===")
    glo = [Grid(m) for _, m in d["/global_costmap/costmap"]]
    maps = [Grid(m) for _, m in d["/map"]]
    print("global grid %dx%d @ %.3f, %d 帧; /map %d 帧" %
          (glo[-1].w, glo[-1].h, glo[-1].res, len(glo), len(maps)))
    # map->odom 未知时，用两次栅格原点差反推机器人位置：全局窗口同样是滚动的
    gg = glo[-1]
    grx, gry = gg.centre()
    print("global 窗口中心 %.2f, %.2f（滚动窗口中心 = 车）" % (grx, gry))

    print("\n-- 全局图沿 +x 走廊（车中线）--")
    for x, counts in corridor_profile(gg, 0.30, 4.00, gry, step=0.20):
        print("  x=%4.2f  free %3d  inf %3d  ins %3d  LETHAL %3d  unk %3d" %
              (x - grx + 0.30, counts["free"], counts["inf"], counts["ins"],
               counts["let"], counts["unk"]))

    for yoff in (0.0, 0.2, -0.2):
        dist, why = first_block(gg, 0.30, 3.95, gry + yoff)
        print("  首次阻挡 y=%+.1f: %s" % (yoff, "无" if dist is None
                                         else "%.2f m (%s)" % (dist, why)))

    # ------------------------------------------------------- phantom latching
    print("\n=== 4. 全局图 phantom 致死格（/map 上不是障碍）===")
    if maps:
        m = maps[-1]
        cls = gg.value()
        lethal_mask = cls == 2
        phantom = 0
        backed = 0
        # 只看车前 0.3-10 m 的所有方向，避免整图外扩
        rows, cols = np.nonzero(lethal_mask)
        for r, c in zip(rows, cols):
            xw = gg.ox + (c + 0.5) * gg.res
            yw = gg.oy + (r + 0.5) * gg.res
            mv = m.cell(xw, yw)
            if mv is None or mv == -1:
                continue
            if mv >= 50:
                backed += 1
            else:
                phantom += 1
        print("  全局图 lethal 格 %d：/map 支撑 %d，phantom %d (%.1f%%)" %
              (len(rows), backed, phantom, 100.0 * phantom / max(1, backed + phantom)))

    # ------------------------------------------------------------------- map
    print("\n=== 5. /map 斑点 ===")
    if maps:
        m = maps[-1]
        occ = m.data >= 50
        free = (m.data >= 0) & (m.data < 50)
        unk = m.data == UNKNOWN
        print("  %dx%d @ %.3f, 占据 %d (%.2f%%), free %d (%.2f%%), unknown %d (%.1f%%)" %
              (m.w, m.h, m.res, occ.sum(), 100.0 * occ.mean(),
               free.sum(), 100.0 * free.mean(), unk.sum(), 100.0 * unk.mean()))
        sizes = components(occ, 4)
        small = [s for s in sizes if s <= 4]
        print("  连通域 %d 个，其中 <=4 px %d 个，占占据像元 %.1f%%" %
              (len(sizes), len(small), 100.0 * sum(small) / max(1, occ.sum())))

    # ------------------------------------------------------------ obstacle scan
    print("\n=== 6. /obstacle_scan 视在净宽 ===")
    scans = d["/obstacle_scan"]
    if scans:
        _, s = scans[-1]
        n = len(s.ranges)
        step = (s.angle_max - s.angle_min) / n
        c = int(round(-s.angle_min / step))
        print("  %d 束, %.3f deg/束, range_min %.2f" % (n, math.degrees(step), s.range_min))

        def bearing_at(deg):
            return math.radians(deg) / step + c

        for band in ((1.4, 2.2), (1.0, 1.6), (2.2, 3.0)):
            hits = []
            for i, r in enumerate(s.ranges):
                if math.isfinite(r) and band[0] <= r <= band[1]:
                    hits.append(math.degrees((i - c) * step))
            if hits:
                print("  %4.1f-%4.1f m 带回波方位: %d 束, %+.1f..%+.1f deg" %
                      (band[0], band[1], len(hits), min(hits), max(hits)))
            else:
                print("  %4.1f-%4.1f m 无回波" % band)

        print("  前方 ±15 deg 内回波距离:")
        for d0 in range(-15, 16, 3):
            i = int(round(bearing_at(d0)))
            if 0 <= i < n:
                r = s.ranges[i]
                print("    %+4d deg  %s" % (d0, ("%.3f m" % r) if math.isfinite(r) else "inf"))

    # ------------------------------------------------------------------- plan
    print("\n=== 7. /plan ===")
    for ts, p in d["/plan"]:
        poses = p.poses
        if not poses:
            print("  t=%.1f  空路径" % ts)
            continue
        pts = [(q.pose.position.x, q.pose.position.y) for q in poses]
        near = [y for x, y in pts if 1.4 <= x <= 2.2]
        print("  t=%.1f  %d 点  %.2f,%.2f -> %.2f,%.2f  窄口带内 |y| 最大 %s" %
              (ts, len(pts), pts[0][0], pts[0][1], pts[-1][0], pts[-1][1],
               "%.2f m" % max(abs(y) for y in near) if near else "n/a"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
