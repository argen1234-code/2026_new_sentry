#!/usr/bin/env python3
"""Run a list of variants back to back and emit a comparison table.

    python3 sweep.py --variants G0,G1,G2,G3,G4,G5,G6
    python3 sweep.py --group G --plant measured
"""

import argparse
import csv
import json
import os
import time

import run_variant as rv

COLS = [
    ("duration_s", "时长 s", "{:.1f}"),
    ("path_length_m", "里程 m", "{:.2f}"),
    ("replan_hz", "重规划 Hz", "{:.2f}"),
    ("jitter_p90_m", "抖动 p90 m", "{:.3f}"),
    ("pairs_over_015_frac", "抖动>0.15 比例", "{:.2f}"),
    ("stops_ge_05s", "停车次数", "{:d}"),
    ("longest_stop_s", "最长停车 s", "{:.2f}"),
    ("cmd_accel_p95_ms2", "指令加速度 p95", "{:.2f}"),
    ("median_clearance_m", "路径最小间距中位 m", "{:.2f}"),
    ("speed_moving_mean_ms", "移动均速 m/s", "{:.3f}"),
]


def fmt(res, key, f):
    if res is None or key not in res or res[key] is None:
        return "-"
    v = res[key]
    try:
        return f.format(v if not isinstance(v, float) else float(v))
    except (ValueError, TypeError):
        return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", default="")
    ap.add_argument("--group", default="")
    ap.add_argument("--defs", default=os.path.join(rv.HERE, "variants.yaml"))
    ap.add_argument("--scenario", default="scenario_a.json")
    ap.add_argument("--bag", default=None)
    ap.add_argument("--plant", default="measured", choices=["measured", "ideal"])
    ap.add_argument("--no-crop", dest="crop", action="store_false")
    ap.add_argument("--map", default=os.path.join(rv.REPO, "maps", "corridor_1025_tape.yaml"))
    ap.add_argument("--out", default="/tmp/replay/results")
    ap.add_argument("--tag", default="")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--stop-on-fail", action="store_true",
                    help="abort the sweep if a variant fails to score")
    args = ap.parse_args()

    scenario_path = args.scenario if os.path.isabs(args.scenario) \
        else os.path.join(rv.HERE, args.scenario)
    scenario = json.load(open(scenario_path))
    if args.bag is None:
        args.bag = scenario["bag"]
    args.goals = scenario["goals"]
    args.rate = 1.0
    args.goal_timeout = 200.0
    args.lifecycle_timeout = 90.0
    args.record_topics = rv.RECORD_TOPICS
    args.manual = False
    args.rviz = False

    running = rv.real_stack_running()
    if running:
        raise SystemExit("refusing to replay: real chassis bridge still running:\n  "
                         + "\n  ".join(running))

    groups = rv.load_defs(args.defs)
    if args.variants:
        names = [n.strip() for n in args.variants.split(",") if n.strip()]
    else:
        prefix = args.group or "G"
        names = [n for n in groups if n.startswith(prefix)]
        names.sort(key=lambda n: (n[0], int(''.join(ch for ch in n if ch.isdigit()) or 0)))

    os.makedirs(args.out, exist_ok=True)
    results = []
    for name in names:
        res = None
        for attempt in (1, 2):
            try:
                res = rv.run_one(name, groups[name], args, args.out)
                break
            except SystemExit as exc:
                rv.log(f"{name} failed (attempt {attempt}): {exc}")
                if attempt == 1:
                    # Give any leaked node time to disappear before trying again;
                    # a busy map_server service name poisons the next attempt.
                    rv.wait_for_quiet()
                    time.sleep(15.0)
        results.append(res)
        if res is None and args.stop_on_fail:
            break

    ref = scenario["reference_metrics"]
    lines = ["| 变体 | 说明 | " + " | ".join(c[1] for c in COLS) + " |",
             "|---|---|" + "---|" * len(COLS)]
    lines.append(f"| 实录基线 | {scenario['bag'].rsplit('/', 1)[-1]} | " +
                 " | ".join(fmt(ref, c[0], c[2]) for c in COLS) + " |")
    for name, res in zip(names, results):
        label = groups[name].get("label", "")
        lines.append(f"| {name} | {label} | " +
                     " | ".join(fmt(res, c[0], c[2]) for c in COLS) + " |")
    table = "\n".join(lines)

    with open(os.path.join(args.out, "sweep.md"), "w") as f:
        f.write(table + "\n")
    with open(os.path.join(args.out, "sweep.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["variant", "label", "plant", "crop"] + [c[0] for c in COLS])
        for name, res in zip(names, results):
            w.writerow([name, groups[name].get("label", ""), args.plant, args.crop]
                       + [(res or {}).get(c[0]) for c in COLS])
    print("\n" + table)


if __name__ == "__main__":
    main()
