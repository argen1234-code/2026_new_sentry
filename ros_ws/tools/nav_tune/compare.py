#!/usr/bin/env python3
"""A/B compare several runs (real bags or replay output) on one table.

The first bag is the reference; every row after it is shown as an absolute value
plus the delta against that reference, so "is variant X better than baseline"
is one glance instead of two reports.

    python3 compare.py ~/bags/before ~/bags/after
    python3 compare.py /tmp/replay/results/B0__.../bag /tmp/replay/results/I1__.../bag ...

Deltas are read relative to the reference, so put the baseline first.
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "replay"))

import common as C  # noqa: E402
import metrics as M  # noqa: E402
import report as R  # noqa: E402
import numpy as np  # noqa: E402

KEYS = [
    ("duration_s", "时长 s", 1),
    ("path_length_m", "里程 m", 1),
    ("replan_hz", "重规划 Hz", 1),
    ("speed_moving_median_ms", "移动中位速 m/s", 1),
    ("jitter_p90_m", "抖动 p90 m", -1),
    ("stops_ge_05s", "停车次数", -1),
    ("longest_stop_s", "最长停车 s", -1),
    ("cmd_accel_p95_ms2", "指令加速度 p95", -1),
    ("serpentine_max_ratio", "蛇形/弦长", -1),
    ("plan_frac_cost_gt0", "路径在代价区比例", -1),
    ("inplace_rotation_frac", "原地转向占比", -1),
    ("min_clearance_m", "最小间距 m", 1),
    ("clearance_p10_m", "路径间距 p10 m", 1),
    ("clearance_below_r_frac", "间距<车体半径占比", -1),
]

GREATER_IS_BETTER = {k: s for k, _, s in KEYS}


def score(bag):
    res = M.analyze(bag)
    d = C.load(bag)
    res.update(R.serpentine(d["plan"]))
    res.update(R.speed_curvature(d["plan"], d["odom"]))
    res.update(R.inplace_rotation(d["odom"]))
    res.update(R.plan_cost(d["plan"], d["gcost"]))
    res.update(R.clearance_stats(d["plan"], d["gcost"], d["odom"]))
    return res


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    return f"{v:.3g}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    results = [score(b) for b in args.bags]
    base = results[0]
    def name_of(path):
        path = os.path.normpath(path)
        # Replay runs record into <run_dir>/bag; the run dir is the useful label.
        if os.path.basename(path) == "bag":
            path = os.path.dirname(path)
        return os.path.basename(path)

    names = [name_of(b) for b in args.bags]

    head = f"{'指标':<18}" + "".join(f"{n[-26:]:>28}" for n in names)
    print(head)
    print("-" * len(head))
    for key, label, sign in KEYS:
        cells = []
        for i, r in enumerate(results):
            v = r.get(key)
            if i == 0 or v is None or base.get(key) in (None, 0):
                cells.append(f"{fmt(v):>28}")
                continue
            delta = (v - base[key]) / abs(base[key]) * 100
            mark = ""
            if sign and abs(delta) >= 3:
                mark = " +" if (delta * sign > 0) else " -"
            cells.append(f"{fmt(v):>22} ({delta:+5.1f}%){mark}")
        print(f"{label:<18}" + "".join(cells))

    print("\n+/- 表示相对第一个 bag 更好/更差（>=3% 才标）；绝对值按同一份代码算出。")
    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump(dict(zip(names, results)), f, indent=2, default=float)


if __name__ == "__main__":
    main()
