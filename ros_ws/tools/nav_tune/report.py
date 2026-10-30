#!/usr/bin/env python3
"""Full behavioural report of one run: what the path looked like, how fast the
robot actually went, and where the two disagree.

Complementary to `replay/metrics.py`, which owns the scalar summary used to
score variants. This adds the shape-level numbers you need to *explain* a bad
score: serpentine amplitude, curvature vs speed, in-place rotation share, and a
per-leg table.

    python3 report.py ~/bags/crash_repro
    python3 report.py <replay bag> --plot /tmp/run.png --json /tmp/run.json
"""

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "replay"))

import common as C  # noqa: E402
import metrics as M  # noqa: E402  (the scoring code the sweeps use)


def curvature(pts, step=5):
    """Smoothed discrete curvature along a polyline: |dtheta/ds| per metre.

    `step` is in path points, so the smoothing distance follows the planner's
    output spacing (~0.1-0.2 m here); it is a shape indicator, not the exact
    number the controller's curvature gate computes from its own lookahead.
    """
    p = np.asarray(pts, dtype=float)
    if len(p) < 2 * step + 1:
        return np.array([])
    a, b, c = p[:-2 * step], p[step:-step], p[2 * step:]
    v1, v2 = b - a, c - b
    n1 = np.linalg.norm(v1, axis=1)
    n2 = np.linalg.norm(v2, axis=1)
    ok = (n1 > 1e-6) & (n2 > 1e-6)
    if not ok.any():
        return np.array([])
    ang = np.arctan2(v1[ok, 1], v1[ok, 0]) - np.arctan2(v2[ok, 1], v2[ok, 0])
    ang = (ang + np.pi) % (2 * np.pi) - np.pi
    ds = 0.5 * (n1[ok] + n2[ok])
    return np.abs(ang) / np.maximum(ds, 1e-6)


def serpentine(plans):
    """How far each plan wanders from its own start->goal chord.

    A planner that is happy with the corridor produces a near-straight line; a
    path decided by tiny differences in a fully-inflated cost field wobbles.
    """
    dev, dev_ratio = [], []
    for _, pts in plans:
        p = np.asarray(pts, dtype=float)
        if len(p) < 5:
            continue
        a, b = p[0], p[-1]
        chord = b - a
        L = np.linalg.norm(chord)
        if L < 1.0:
            continue
        n = np.array([-chord[1], chord[0]]) / L
        d = np.abs((p - a) @ n)
        # Only the middle counts: the endpoints are pinned by definition.
        mid = d[len(d) // 5: -max(1, len(d) // 5)]
        if mid.size:
            dev.append(float(mid.max()))
            dev_ratio.append(float(mid.max() / L))
    if not dev:
        return {}
    return {"serpentine_max_m": round(float(np.median(dev)), 3),
            "serpentine_max_ratio": round(float(np.median(dev_ratio)), 3),
            "serpentine_max_p95_ratio": round(float(np.percentile(dev_ratio, 95)), 3)}


def speed_curvature(plans, odom):
    """Speed the robot actually travels where its plan is curved.

    Each odometry sample looks up the curvature at the nearest point of the plan
    current at that time -- pairing speed with the shape the controller was
    tracking, not with one speed per plan (which makes every bucket identical).
    """
    if not plans or not odom:
        return {}
    ks, ss = [], []
    for o in odom:
        t = o[0]
        _, pts = C.nearest(plans, t)
        p = np.asarray(pts, dtype=float)
        if len(p) < 12:
            continue
        k = curvature(pts)
        if not len(k):
            continue
        d = np.hypot(p[:, 0] - o[1], p[:, 1] - o[2])
        # curvature() drops `step` points from each end
        idx = int(np.argmin(d)) - 5
        if 0 <= idx < len(k):
            ks.append(float(k[idx]))
            ss.append(math.hypot(o[4], o[5]))
    if not ks:
        return {}
    ks, ss = np.array(ks), np.array(ss)
    # Only moving samples answer "does curvature slow the robot down?"; idle
    # stretches (waiting for the next goal) would otherwise all land in the
    # straight-ahead bucket at speed 0.
    moving = ss > 0.05
    out = {"curvature_median_per_m": round(float(np.median(ks[moving])), 3)
           if moving.any() else None}
    for lo, hi in ((0.0, 0.4), (0.4, 0.7), (0.7, 1e9)):
        sel = moving & (ks >= lo) & (ks < hi)
        name = f"speed_at_curv_{lo:g}_{hi:g}"
        out[name] = round(float(np.median(ss[sel])), 3) if sel.sum() > 20 else None
        out[name + "_n"] = int(sel.sum())
    return out


def inplace_rotation(odom, win=0.4):
    """Share of time the robot turns while barely translating.

    Rates come from pose differences over a `win` window, not from the twist
    field: the recorded twist is time-shifted against the pose, and a live
    chassis' yaw dithers sample to sample.
    """
    if len(odom) < 5:
        return {}
    t = np.array([r[0] for r in odom])
    x = np.array([r[1] for r in odom])
    y = np.array([r[2] for r in odom])
    yaw = np.unwrap([r[3] for r in odom])
    dt, wz, wsign, v = [], [], [], []
    j = 0
    for i in range(len(t)):
        while j < i and t[i] - t[j] > win:
            j += 1
        d = t[i] - t[j]
        if d < 1e-3:
            dt.append(0.0)
            wz.append(0.0)
            wsign.append(0.0)
            v.append(0.0)
            continue
        rate = (yaw[i] - yaw[j]) / d
        dt.append(d)
        wz.append(abs(rate))
        wsign.append(rate)
        v.append(math.hypot(x[i] - x[j], y[i] - y[j]) / d)
    dt, wz, v = np.array(dt), np.array(wz), np.array(v)
    mask = (wz > 0.25) & (v < 0.06)
    # Count a reversal only when both sides are a real turn, so window noise is
    # not mistaken for a direction change.
    sign = np.where(wz > 0.25, np.sign(wsign), 0)
    flips = 0
    prev = 0
    for s in sign:
        if s != 0:
            if prev != 0 and s != prev:
                flips += 1
            prev = s
    return {"inplace_rotation_frac": round(float((mask * dt).sum() / dt.sum()), 3),
            "yaw_reversals": flips}


def legs(plans, odom):
    """Split the run at goal changes (the plan endpoint jumps) and score each."""
    if len(plans) < 2:
        return []
    out, start, goal = [], plans[0][0], plans[0][1][-1]
    for t, pts in plans[1:]:
        g = pts[-1]
        if math.hypot(g[0] - goal[0], g[1] - goal[1]) > 1.0:
            out.append((start, t, goal))
            start, goal = t, g
    out.append((start, plans[-1][0], goal))
    rows = []
    for t0, t1, g in out:
        xs = [(r[0], r[1], r[2]) for r in odom if t0 <= r[0] <= t1]
        if not xs:
            continue
        dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(xs, xs[1:]))
        dt = max(t1 - t0, 1e-6)
        rows.append({"t0": round(t0, 1), "dur_s": round(dt, 1),
                     "path_m": round(dist, 1), "mean_speed": round(dist / dt, 3),
                     "goal": [round(g[0], 2), round(g[1], 2)]})
    return rows


def controller_divergence(d):
    """cmd_vel_controller (plugin output) vs cmd_vel (post-smoother)."""
    if not d["cmd_ctrl"] or not d["cmd"]:
        return {}
    diffs = []
    for t, vx, vy, wz in d["cmd_ctrl"]:
        if not (d["cmd"][0][0] <= t <= d["cmd"][-1][0]):
            continue
        s = C.pose_at(d["cmd"], t)
        diffs.append(math.hypot(vx - s[1], vy - s[2]))
    if not diffs:
        return {}
    a = np.array(diffs)
    return {"ctrl_vs_smoothed_p50": round(float(np.median(a)), 3),
            "ctrl_vs_smoothed_p95": round(float(np.percentile(a, 95)), 3)}


def plan_cost(plans, gcost):
    """Cost the planner's own path sits on (0-100 published scale)."""
    if not plans or not gcost:
        return {}
    vals = []
    for t, pts in plans[:: max(1, len(plans) // 40)]:
        msg = C.nearest(gcost, t)[1]
        g = C.grid_of(msg)
        for x, y in pts:
            rc = C.cell_of(msg, x, y)
            if rc is not None:
                vals.append(int(g[rc]))
    if not vals:
        return {}
    a = np.array(vals)
    return {"plan_cost_median": int(np.median(a)),
            "plan_cost_p95": int(np.percentile(a, 95)),
            "plan_frac_cost_gt0": round(float((a > 0).mean()), 3)}


def clearance_stats(plans, gcost, odom=None, robot_radius=0.26, skip_r=0.6):
    """How close the planned path comes to lethal cells, as a distribution.

    metrics.plan_clearance reports the run-wide minimum, and that number is 0
    on every run -- one frame where a phantom lethal cell flickers next to the
    robot is enough. It cannot answer "did this variant start cutting corners".
    This reports per plan: the distance from the closest path point to the
    nearest lethal cell, then the p10 / median of those, plus the fraction of
    plans that come inside the robot radius at all.

    Path points within skip_r of the robot are dropped. The known phantom
    lethal cells flicker around the robot's own footprint (see
    docs/lethal_flicker_fix.md), so including the path's start would measure
    that artefact instead of how close the route runs to an obstacle.
    """
    if not plans or not gcost:
        return {}
    pts_all = []
    for t, msg in gcost:
        lethal = M.lethal_points(msg)
        if lethal is None:
            continue
        pts = np.asarray(C.nearest(plans, t)[1])
        if len(pts) == 0 or odom is None:
            continue
        o = C.pose_at(odom, t)
        if o is None or len(o) < 5:
            continue
        pts = pts[np.hypot(pts[:, 0] - o[1], pts[:, 1] - o[2]) > skip_r]
        if len(pts) == 0:
            continue
        # Chunked so a 2000x60 distance matrix stays cheap.
        d = np.min(
            np.concatenate([
                np.hypot(pts[:, None, 0] - lethal[None, i:i + 256, 0],
                         pts[:, None, 1] - lethal[None, i:i + 256, 1]).min(axis=1, keepdims=True)
                for i in range(0, len(lethal), 256)]), axis=1)
        pts_all.append(float(d.min()))
    if not pts_all:
        return {}
    a = np.array(pts_all)
    return {"clearance_p10_m": round(float(np.percentile(a, 10)), 3),
            "clearance_median_m": round(float(np.median(a)), 3),
            "clearance_below_r_frac": round(float((a < robot_radius).mean()), 3)}


def plot(path, d, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(13, 8))

    a = ax[0][0]
    if d["gcost"]:
        msg = d["gcost"][len(d["gcost"]) // 2][1]
        g = C.grid_of(msg)
        ext = [msg.info.origin.position.x,
               msg.info.origin.position.x + msg.info.width * msg.info.resolution,
               msg.info.origin.position.y,
               msg.info.origin.position.y + msg.info.height * msg.info.resolution]
        a.imshow(g, origin="lower", extent=ext, cmap="magma", vmin=0, vmax=100)
    if d["odom"]:
        a.plot([r[1] for r in d["odom"]], [r[2] for r in d["odom"]],
               "-", color="cyan", lw=1.2, label="odom")
    if d["plan"]:
        p = np.asarray(d["plan"][len(d["plan"]) // 2][1])
        a.plot(p[:, 0], p[:, 1], "-", color="lime", lw=1.0, label="plan @mid")
    a.set_title("trajectory over global costmap")
    a.legend(loc="upper right", fontsize=8)
    a.set_aspect("equal")

    a = ax[0][1]
    if d["odom"]:
        t = np.array([r[0] for r in d["odom"]])
        v = np.array([math.hypot(r[4], r[5]) for r in d["odom"]])
        a.plot(t - t[0], v, lw=0.8, label="odom |v|")
    if d["cmd"]:
        t = np.array([r[0] for r in d["cmd"]])
        a.plot(t - t[0], [math.hypot(r[1], r[2]) for r in d["cmd"]],
               lw=0.6, alpha=0.7, label="cmd |v|")
    a.set_xlabel("t [s]")
    a.set_ylabel("m/s")
    a.set_title("speed: commanded vs actual")
    a.legend(fontsize=8)

    a = ax[1][0]
    if d["plan"]:
        t = np.array([p[0] for p in d["plan"]])
        dev = []
        for _, pts in d["plan"]:
            p = np.asarray(pts, dtype=float)
            if len(p) < 5:
                dev.append(np.nan)
                continue
            ch = p[-1] - p[0]
            L = np.linalg.norm(ch)
            n = np.array([-ch[1], ch[0]]) / max(L, 1e-6)
            mid = np.abs((p - p[0]) @ n)[len(p) // 5: -max(1, len(p) // 5)]
            dev.append(float(mid.max()) if mid.size else np.nan)
        a.plot(t - t[0], dev, ".", ms=2)
        a.set_xlabel("t [s]")
        a.set_ylabel("m")
        a.set_title("plan wander from its own chord (per plan)")

    a = ax[1][1]
    if d["plan"]:
        allk = [curvature(pts) for _, pts in d["plan"]]
        allk = [k for k in allk if len(k)]
        if allk:
            a.hist(np.concatenate(allk), bins=60, range=(0, 2.0))
        a.set_xlabel("|curvature| 1/m")
        a.set_title("plan curvature distribution")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"图已存 {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--json", default=None)
    ap.add_argument("--plot", default=None)
    ap.add_argument("--no-legs", dest="legs", action="store_false")
    args = ap.parse_args()

    res = M.analyze(args.bag)
    d = C.load(args.bag)
    res.update(serpentine(d["plan"]))
    res.update(speed_curvature(d["plan"], d["odom"]))
    res.update(inplace_rotation(d["odom"]))
    res.update(plan_cost(d["plan"], d["gcost"]))
    res.update(controller_divergence(d))
    if args.legs:
        lg = legs(d["plan"], d["odom"])
        if lg:
            res["legs"] = lg

    keys = [k for k in res if k != "legs"]
    width = max(len(k) for k in keys)
    print(f"=== {args.bag}")
    for k in keys:
        print(f"  {k.ljust(width)}  {res[k]}")
    if res.get("legs"):
        print("\n  ---- 分段（按目标切换切分）----")
        print(C.table([[str(r["t0"]), str(r["dur_s"]), str(r["path_m"]),
                        str(r["mean_speed"]), str(r["goal"])] for r in res["legs"]],
                      ("t0", "时长", "里程", "均速", "目标")))

    if args.json:
        with open(args.json, "w") as f:
            import json
            json.dump(res, f, indent=2)
        print(f"\nJSON 已存 {args.json}")
    if args.plot:
        plot(args.plot, d, res)


if __name__ == "__main__":
    main()
