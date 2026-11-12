#!/usr/bin/env python3
"""Composite score for one navigation run, or a set of runs of the same route.

    python3 score.py BAG [BAG ...] [--json out.json] [--label NAME] [--no-doctor]

Why this exists. `replay/an_run.py` answers "did the command reach the wheels",
`report.py` answers "what shape did the run have", `compare.py` puts two bags in
one table -- but none of them turns a whole multi-leg route into one number that
can be ranked across parameter sets. This adds only that:

  * leg segmentation (by /goal_pose when the bag has it, else by /plan
    endpoint changes -- the fallback is what the 11-12 bags need),
  * `yaw_err_deg`, which an_run.py does not compute,
  * the normalisation table and the weights.

Everything else is the existing code: `common.load` reads the bag,
`an_run`'s dead-command test is reproduced here restricted to a leg, `metrics`
supplies the clearance sampling, `report.clearance_stats` supplies the
run-wide clearance (its min_clearance is 0 on nearly every run because a
phantom lethal cell flickers next to the robot -- see docs/lethal_flicker_fix.md).

Weights, agreed 2026-11-12: efficiency 25 / accuracy 25 / reliability 25 /
smoothness 15 / safety 10. A collision or a doctor.py FAIL zeroes the run.
"""

import argparse
import bisect
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "replay"))

import common as C  # noqa: E402
import metrics as M  # noqa: E402
import report as R  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

WEIGHTS = {"accuracy": 25.0, "reliability": 25.0, "efficiency": 25.0,
           "smoothness": 15.0, "safety": 10.0}

# Velocity ceilings the efficiency index is measured against: the controller's
# own caps (nav2_params.yaml FollowPath.v_linear_max / v_angular_max).
V_CAP = 0.45
W_CAP = 0.8

# Normalisation: (full marks, still passing). Linear between, clamped outside.
# `higher` = the larger the number the better.
NORM = {
    "final_err_m":  (0.05, 0.20, False),
    "yaw_err_deg":  (2.0, 10.0, False),
    "lat_p90_m":    (0.03, 0.10, False),
    "dead_cmd_s":   (0.0, 5.0, False),
    "repeat_span_m": (0.03, 0.15, False),
    "success_frac": (1.0, 0.5, True),
    "eff_ratio":    (0.65, 0.25, True),
    "cmd_accel_p95": (1.5, 3.0, False),
    "v_swing_frac": (0.0, 0.5, False),
    "clearance_p10_m": (0.30, 0.15, True),
}


def ramp(value, full, passing, higher):
    """1.0 at or beyond `full`, 0.0 at or beyond `passing`, linear between.

    Returns None for a missing number so the caller can drop the sub-metric
    instead of scoring it zero -- "no data" is not "bad".
    """
    if value is None:
        return None
    if higher:
        if value >= full:
            return 1.0
        if value <= passing:
            return 0.0
        return (value - passing) / (full - passing)
    if value <= full:
        return 1.0
    if value >= passing:
        return 0.0
    return (passing - value) / (passing - full)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_goals(path):
    """[(t, x, y, yaw)] from /goal_pose; empty when the bag has no such topic.

    RViz's 2D Goal Pose publishes this, so goals clicked by hand are recorded.
    Goals sent through the NavigateToPose *action* are not (the 11-12 bags).
    """
    try:
        r = C.reader(path, ["/goal_pose"])
    except Exception:
        return []
    out = []
    while r.has_next():
        _, data, ts = r.read_next()
        m = deserialize_message(data, PoseStamped)
        p, q = m.pose.position, m.pose.orientation
        out.append((ts * 1e-9, p.x, p.y, C.yaw_of(q)))
    out.sort(key=lambda g: g[0])
    return out


def load_gate(path):
    """[(t, enabled)] from /unified_cmd's CMD_MOTION_ENABLE bit (0x0002)."""
    try:
        r = C.reader(path, ["/unified_cmd"])
    except Exception:
        return None
    from sentry_interfaces.msg import UnifiedCmd
    out = []
    while r.has_next():
        _, data, ts = r.read_next()
        m = deserialize_message(data, UnifiedCmd)
        out.append((ts * 1e-9, bool(m.chassis_status & 0x0002)))
    out.sort(key=lambda x: x[0])
    return out or None


def segments(goals, plans, t_end):
    """[(t_start, t_end, (gx, gy, gyaw) | None)] -- one entry per goal.

    Without /goal_pose, split where the plan's endpoint jumps: that is where a
    new goal took over. A pure rotation goal produces no jump and collapses
    into one leg, which is correct -- it is one goal.
    """
    if goals:
        bounds = [g[0] for g in goals] + [t_end]
        return [(goals[i][0], bounds[i + 1], (goals[i][1], goals[i][2], goals[i][3]))
                for i in range(len(goals))]
    if not plans:
        return []
    starts, cur = [], None
    for t, pts in plans:
        e = pts[-1]
        if cur is None or math.hypot(e[0] - cur[0], e[1] - cur[1]) > 0.30:
            starts.append(t)
            cur = e
    if not starts:
        return [(plans[0][0], t_end, None)]
    starts[0] = min(starts[0], plans[0][0])
    starts.append(t_end)
    return [(starts[i], starts[i + 1], None) for i in range(len(starts) - 1)]


def cmd_accel_dedup(cmds):
    """Acceleration between *distinct* command values.

    /cmd_vel is republished at 100 Hz by fake_vel_transform while the
    controller updates at 20 Hz, so five consecutive frames carry one value and
    the whole step lands in a single 10 ms interval -- metrics.cmd_accel then
    reports ~5x the real acceleration (docs/nav2_tuning.md:259-274).
    """
    uniq = []
    for c in cmds:
        if uniq and c[1] == uniq[-1][1] and c[2] == uniq[-1][2] and c[3] == uniq[-1][3]:
            continue
        uniq.append(c)
    accs = []
    for a, b in zip(uniq, uniq[1:]):
        dt = b[0] - a[0]
        if dt <= 1e-4 or dt > 0.5:
            continue
        accs.append(math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
    return float(np.percentile(accs, 95)) if accs else 0.0


def dead_time(cmds, odom, gate=None):
    """Seconds with a non-zero command while the robot does not move.

    Same test as an_run.py:120-152 (0.5 s pose window, <0.05 m, <0.1 rad), so
    the numbers stay comparable with the archive in ~/bags/tune1111.

    `gate` is [(t, enabled)] from /unified_cmd: samples outside an enabled
    window are dropped. Without it the "recorder was running before someone
    flipped enable_motion" stretch reads as 26 s of chassis deadband -- which is
    what it did on 11-12 18:21, an operator artefact, not a motor.
    """
    if not cmds or not odom:
        return 0.0
    flags = []
    for c in cmds:
        if gate is not None:
            ge = C.nearest(gate, c[0])
            if not ge[1]:
                flags.append(False)
                continue
        active = math.hypot(c[1], c[2]) > 0.02 or abs(c[3]) > 0.02
        a, b = C.nearest(odom, c[0] - 0.5), C.nearest(odom, c[0] + 0.5)
        dyaw = abs(wrap(b[3] - a[3]))
        still = math.hypot(b[1] - a[1], b[2] - a[2]) < 0.05 and dyaw < 0.1
        flags.append(bool(active and still))
    total, start, last = 0.0, None, None
    for i in range(1, len(cmds)):
        if flags[i]:
            if start is None:
                start = cmds[i - 1][0]
            last = cmds[i][0]
        elif start is not None:
            if last - start >= 0.5:
                total += last - start
            start = None
    if start is not None and last - start >= 0.5:
        total += last - start
    return total


def leg_metrics(t0, t1, goal, data, gate=None):
    """Per-leg numbers. Returns None when the leg has no motion and no goal."""
    odom = [o for o in data["odom"] if t0 <= o[0] < t1]
    cmds = [c for c in data["cmd"] if t0 <= c[0] < t1]
    plans = [p for p in data["plan"] if t0 <= p[0] < t1]
    moving = [c for c in cmds if math.hypot(c[1], c[2]) > 0.02 or abs(c[3]) > 0.02]
    if not odom or (not moving and goal is None):
        return None

    poses = [(t, *C.map_pose(data, t)) for t, *_ in odom]
    first_mv = moving[0][0] if moving else t0
    last_mv = moving[-1][0] if moving else t1
    # Active duration: first to last non-zero command. Trailing idle frames
    # (the recorder keeps running) must not count as slowness.
    active_s = max(last_mv - first_mv, 1e-3)

    x0, y0, yaw0 = poses[0][1], poses[0][2], poses[0][3]
    xe, ye, yawe = poses[-1][1], poses[-1][2], poses[-1][3]

    # Path length and heading change actually travelled, for the efficiency index.
    # Per-step yaw below 0.5 deg is odometry noise, not rotation; summing it over
    # thousands of 50 Hz samples would invent seconds of ideal rotation time.
    path_len = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))
    dyaw = 0.0
    for a, b in zip(poses, poses[1:]):
        step = abs(wrap(b[3] - a[3]))
        if step > math.radians(0.5):
            dyaw += step
    ideal_s = path_len / V_CAP + dyaw / W_CAP

    out = {"active_s": round(active_s, 2),
           "path_len_m": round(path_len, 2),
           "eff_ratio": round(min(ideal_s / active_s, 1.5), 2) if active_s > 0 else 0.0,
           "dead_cmd_s": round(dead_time(cmds, odom, gate), 1)}

    if goal is not None:
        gx, gy, gyaw = goal
        out["final_err_m"] = round(math.hypot(gx - xe, gy - ye), 3)
        out["yaw_err_deg"] = round(abs(math.degrees(wrap(gyaw - yawe))), 1)
    elif plans:
        gx, gy = plans[-1][1][-1]
        out["final_err_m"] = round(math.hypot(gx - xe, gy - ye), 3)
        out["yaw_err_deg"] = None
    else:
        out["final_err_m"] = None
        out["yaw_err_deg"] = None

    # Lateral error, map frame, only while a translation command was live.
    moving_t = [c[0] for c in moving]
    lat = []
    if plans and moving_t:
        for t, px, py, _ in poses:
            i = bisect.bisect_left(moving_t, t)
            i = min(i, len(moving_t) - 1)
            if abs(moving_t[i] - t) > 0.25:
                continue
            path = C.nearest(plans, t)[1]
            lat.append(min(math.hypot(a - px, b - py) for a, b in path))
    out["lat_p90_m"] = round(float(np.percentile(lat, 90)), 3) if len(lat) > 10 else None
    out["_yaw_change_deg"] = round(math.degrees(dyaw), 1)
    out["_start"] = [round(x0, 2), round(y0, 2), round(math.degrees(yaw0), 1)]
    return out


def doctor_fails(path):
    """Run doctor.py and return its FAIL count (0 when it cannot run)."""
    exe = os.path.join(HERE, "doctor.py")
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        tmp = f.name
    try:
        subprocess.run([sys.executable, exe, path, "--json", tmp],
                       capture_output=True, timeout=300)
        with open(tmp) as f:
            res = json.load(f)
        return [c for c in res["checks"] if c["status"] == "FAIL"]
    except Exception:
        return []
    finally:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def analyze_run(path, use_doctor=True):
    data = C.load(path)
    if not data["odom"] or not data["cmd"]:
        raise SystemExit(f"{path}: 需要 /odometry 与 /cmd_vel")
    goals = load_goals(path)
    t_end = max(data["odom"][-1][0], data["cmd"][-1][0])
    segs = segments(goals, data["plan"], t_end)
    gate = load_gate(path)

    legs = [m for m in (leg_metrics(a, b, g, data, gate) for a, b, g in segs) if m]
    run = {"bag": path, "legs": legs}
    if not legs:
        run["veto"] = ["no-legs"]
        run["score"] = 0.0
        return run

    def vals(k):
        return [l[k] for l in legs if l.get(k) is not None]

    run["final_err_m"] = round(float(np.mean(vals("final_err_m"))), 3) if vals("final_err_m") else None
    run["final_err_max_m"] = round(float(max(vals("final_err_m"))), 3) if vals("final_err_m") else None
    run["yaw_err_deg"] = round(float(np.mean(vals("yaw_err_deg"))), 1) if vals("yaw_err_deg") else None
    run["lat_p90_m"] = round(float(np.mean(vals("lat_p90_m"))), 3) if vals("lat_p90_m") else None
    run["dead_cmd_s"] = round(sum(l["dead_cmd_s"] for l in legs), 1)
    run["eff_ratio"] = round(float(np.mean(vals("eff_ratio"))), 2)
    run["active_s"] = round(sum(l["active_s"] for l in legs), 1)
    run["success_frac"] = round(float(np.mean([e is not None and e <= 0.25
                                              for e in vals("final_err_m")])), 2)
    run["cmd_accel_p95"] = round(cmd_accel_dedup(data["cmd"]), 2)
    cv = [math.hypot(c[1], c[2]) for c in data["cmd"]
          if math.hypot(c[1], c[2]) > 0.02]
    run["v_swing_frac"] = round(float(np.mean([0.15 <= v < 0.35 for v in cv])), 2) if cv else 0.0

    cs = R.clearance_stats(data["plan"], data["gcost"], data["odom"])
    run["clearance_p10_m"] = cs.get("clearance_p10_m")

    run["veto"] = []
    if run["clearance_p10_m"] is not None and run["clearance_p10_m"] < 0.05:
        run["veto"].append("clearance<0.05m")
    if use_doctor:
        fails = doctor_fails(path)
        if fails:
            run["veto"].append("doctor:" + ",".join(c["name"] for c in fails))
    return run


def mean_known(items):
    """Average of the sub-metrics that are present.

    A missing number means the bag cannot answer the question (/goal_pose was
    never recorded, the route never came within 0.6 m of a lethal cell); scoring
    it as 0 would confuse "unknown" with "bad" and make such runs unrankable.
    """
    v = [x for x in items if x is not None]
    return float(np.mean(v)) if v else None


def score_run(run, repeat_span=None):
    if run.get("veto"):
        run["dims"] = {k: 0.0 for k in WEIGHTS}
        run["score"] = 0.0
        return run
    dims = {
        "accuracy": mean_known([
            ramp(run.get("final_err_m"), *NORM["final_err_m"]),
            ramp(run.get("yaw_err_deg"), *NORM["yaw_err_deg"]),
            ramp(run.get("lat_p90_m"), *NORM["lat_p90_m"])]),
        "reliability": mean_known([
            ramp(run.get("dead_cmd_s"), *NORM["dead_cmd_s"]),
            ramp(repeat_span, *NORM["repeat_span_m"]),
            ramp(run.get("success_frac"), *NORM["success_frac"])]),
        "efficiency": ramp(run.get("eff_ratio"), *NORM["eff_ratio"]),
        # v_swing_frac is reported but not scored: on a short leg the whole run
        # sits inside approach_velocity_scaling_dist, so it measures the leg
        # length rather than the controller.
        "smoothness": ramp(run.get("cmd_accel_p95"), *NORM["cmd_accel_p95"]),
        "safety": ramp(run.get("clearance_p10_m"), *NORM["clearance_p10_m"]),
    }
    missing = [k for k, v in dims.items() if v is None]
    total = sum(WEIGHTS[k] for k, v in dims.items() if v is not None)
    raw = (round(sum(WEIGHTS[k] * v for k, v in dims.items()
                     if v is not None) / total * 100.0, 1) if total else 0.0)
    # A failed run keeps score 0 so it can never outrank one that arrived, but
    # `raw` keeps the underlying numbers visible for diagnosis.
    run["score_raw"] = raw
    run["score"] = 0.0 if run.get("veto") else raw
    run["dims"] = {k: (None if v is None else round(v, 3)) for k, v in dims.items()}
    run["missing"] = missing
    return run


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--json", default=None)
    ap.add_argument("--label", default=None)
    ap.add_argument("--no-doctor", dest="doctor", action="store_false")
    ap.add_argument("--repeat", action="store_true",
                    help="这些 bag 是同一路线重复跑；只在这时才算重复性")
    args = ap.parse_args()

    runs = [analyze_run(b, args.doctor) for b in args.bags]

    # Repeatability: spread of the final error for the *same* leg across laps.
    # Pooling every leg would subtract the outbound error from the inbound one,
    # which measures the route, not the repeatability.
    span = None
    if args.repeat and len(runs) > 1:
        spans = []
        for i in range(min(len(r["legs"]) for r in runs)):
            errs = [r["legs"][i].get("final_err_m") for r in runs]
            errs = [e for e in errs if e is not None]
            if len(errs) > 1:
                spans.append(max(errs) - min(errs))
        if spans:
            span = round(float(max(spans)), 3)
    for r in runs:
        score_run(r, span)

    label = args.label or os.path.basename(os.path.normpath(args.bags[0]))
    head = ("# 导航跑分  " + label)
    rows = []
    for r in runs:
        rows.append((os.path.basename(os.path.normpath(r["bag"])),
                     f'{r["score"]:.1f}', f'{r.get("final_err_m")}',
                     f'{r.get("yaw_err_deg")}', f'{r.get("lat_p90_m")}',
                     f'{r.get("dead_cmd_s")}', f'{r.get("eff_ratio")}',
                     f'{r.get("cmd_accel_p95")}', f'{r.get("v_swing_frac")}',
                     f'{r.get("clearance_p10_m")}',
                     ";".join(r["veto"]) or "-"))
    print(head)
    print(C.table(rows, ("bag", "score", "err_m", "yaw_deg", "lat_p90", "dead_s",
                         "eff", "acc_p95", "swing", "clear_p10", "veto")))
    print(f"\n权重 {WEIGHTS}；重复性极差 {span} m（{len(errs)} 段）")
    for r in runs:
        print(f'  {os.path.basename(os.path.normpath(r["bag"]))}: {r.get("dims")}')

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"label": label, "weights": WEIGHTS,
                       "repeat_span_m": span, "runs": runs}, f,
                      ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
