#!/usr/bin/env python3
"""Offline health check of a nav2 run, from its bag.

Answers the question "was the stack itself healthy?" before any tuning verdict:
are the topics alive, is the robot ever inside a lethal cell, is a command that
never produced motion, did a costmap stop updating, did the planner go quiet.

    python3 doctor.py ~/bags/crash_repro
    python3 doctor.py /tmp/replay/results/B0__.../bag --robot-radius 0.30

Exit code is 1 if any check FAILs, so it can gate a sweep.
"""

import argparse
import math
import sys

import common as C


def check(rows, name, status, detail, hint=""):
    rows.append((status, name, detail, hint))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--robot-radius", type=float, default=0.30,
                    help="半径内出现致命格只作提示（代价地图有幽灵致命格）")
    ap.add_argument("--contact-radius", type=float, default=0.12,
                    help="该半径内出现致命格判为接触：规划器会拒绝起点")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    d = C.load(args.bag)
    counts = d["counts"]
    rows = []
    if not d["odom"]:
        raise SystemExit(f"no /odometry in {args.bag}")

    t0 = d["odom"][0][0]
    span = d["odom"][-1][0] - t0

    # --- 1. topics -----------------------------------------------------------
    expect = {C.ODOM: 10, C.PLAN: 0.5, C.CMD: 10, C.GLOBAL_COSTMAP: 0.5,
              C.LOCAL_COSTMAP: 0.5}
    for topic, min_hz in expect.items():
        if topic not in counts:
            check(rows, f"topic {topic}", "FAIL", "未录到", "确认录制列表/节点是否起来")
            continue
        n, a, b = counts[topic]
        hz = C.rate(n, a, b)
        status = "OK" if hz >= min_hz else ("WARN" if hz > 0 else "FAIL")
        check(rows, f"topic {topic}", status, f"{n} 条, {hz:.2f} Hz")

    # --- 2. odometry continuity ---------------------------------------------
    gaps = [(b[0] - a[0], a[0] - t0) for a, b in zip(d["odom"], d["odom"][1:])]
    worst = max(gaps) if gaps else (0.0, 0.0)
    check(rows, "odom 连续性", "OK" if worst[0] < 0.3 else "WARN",
          f"最大间隔 {worst[0]:.2f} s @ t={worst[1]:.1f}s")

    # --- 3. robot inside lethal cells (the collision signature) --------------
    # Two rings, because they mean different things:
    #   contact  -- a lethal cell under the robot centre. The theta star start
    #               cell is lethal, the planner refuses every goal and nav2
    #               cannot recover on its own. This is the crash signature.
    #   near     -- lethal within the footprint. The costmap is known to flicker
    #               phantom lethal cells around the robot (docs/lethal_flicker_fix.md),
    #               so this is a warning to look at, not proof of contact.
    if d["gcost"] and d["gframe"] == "map" and not d["map2odom"]:
        check(rows, "车体中心格是否致命", "SKIP",
              "全局代价地图在 map 系但包里没有 map->odom，位姿不可比")
    elif not d["gcost"]:
        pass
    else:
        center, near, far, streak, best = [], [], [], 0, None
        for t, msg in d["gcost"]:
            x, y, _ = C.map_pose(d, t)
            rc = C.cell_of(msg, x, y)
            if rc is None:
                continue
            g = C.grid_of(msg)
            hit = int(g[rc]) >= C.LETHAL_PUBLISHED
            if hit:
                center.append(t - t0)
                streak += 1
                if best is None or streak > best[0]:
                    best = (streak, t - t0, x, y)
            else:
                streak = 0
            if C.max_cost_near(msg, x, y, args.contact_radius) >= C.LETHAL_PUBLISHED:
                near.append(t - t0)
            if C.max_cost_near(msg, x, y, args.robot_radius) >= C.LETHAL_PUBLISHED:
                far.append(t - t0)
        n_cost = len(d["gcost"])
        if center:
            check(rows, "车体中心格是否致命", "FAIL",
                  f"{len(center)}/{n_cost} 帧, 最长 {best[0] * 0.5:.1f} s "
                  f"@ t={best[1]:.1f}s ({best[2]:.2f},{best[3]:.2f})",
                  "已接触：规划器会拒绝起点，nav2 无法自恢复")
        else:
            check(rows, "车体中心格是否致命", "OK", f"{n_cost} 帧均干净")
        frac = len(near) / max(n_cost, 1)
        check(rows, f"{args.contact_radius:.2f} m 内出现致命格",
              "WARN" if frac > 0.10 else "OK", f"{len(near)}/{n_cost} 帧 ({frac:.0%})",
              "已知代价地图有瞬态幽灵致命格（docs/lethal_flicker_fix.md）；"
              "连续多帧成片则要查自车点/地面点")
        frac = len(far) / max(n_cost, 1)
        check(rows, f"{args.robot_radius:.2f} m 内出现致命格", "OK",
              f"{len(far)}/{n_cost} 帧 ({frac:.0%})",
              "正常：贴着墙/障碍走就会这样")

    # --- 4. commanded but not moving ----------------------------------------
    # Only translation counts: standing still while spinning is rotate_to_heading.
    episodes, run_start, origin = [], None, None
    for t, vx, vy, wz in d["cmd"]:
        o = C.pose_at(d["odom"], t)
        if math.hypot(vx, vy) > 0.20 and o[4] < 0.05 and o[5] < 0.05:
            if run_start is None:
                run_start, origin = t, (o[1], o[2])
        elif run_start is not None:
            if t - run_start > 1.5:
                episodes.append((run_start - t0, t - run_start, origin))
            run_start = None
    if run_start is not None and d["cmd"][-1][0] - run_start > 1.5:
        episodes.append((run_start - t0, d["cmd"][-1][0] - run_start, origin))
    if episodes:
        longest = max(episodes, key=lambda e: e[1])
        check(rows, "有指令但不动", "WARN",
              f"{len(episodes)} 段, 最长 {longest[1]:.1f} s @ t={longest[0]:.1f}s "
              f"({longest[2][0]:.2f},{longest[2][1]:.2f})",
              "低于底盘死区被吃掉 / 车被挡住 / 局部代价太高")
    else:
        check(rows, "有指令但不动", "OK", "未发现")

    # --- 5. costmap / plan liveness -----------------------------------------
    for name, series in (("global_costmap", d["gcost"]), ("local_costmap", d["lcost"]),
                         ("plan", d["plan"])):
        if len(series) < 2:
            check(rows, f"{name} 更新", "WARN", f"{len(series)} 条")
            continue
        gaps = [(b[0] - a[0], a[0] - t0) for a, b in zip(series, series[1:])]
        worst = max(gaps)
        check(rows, f"{name} 更新", "OK" if worst[0] < 2.0 else "WARN",
              f"最大间断 {worst[0]:.2f} s @ t={worst[1]:.1f}s")

    # --- 6. TF ---------------------------------------------------------------
    if d["map2odom"]:
        z = [m[3] for m in d["map2odom"]]
        check(rows, "map->odom", "OK",
              f"z∈[{min(z):.3f},{max(z):.3f}] 末帧 {z[-1]:.3f}")
    elif d["gframe"] == "map":
        check(rows, "map->odom", "FAIL", "全局代价地图在 map 系但包里没有 map->odom",
              "无法把车体位姿放进地图系，碰撞判据失效")

    print(f"=== {args.bag}  时长 {span:.1f} s")
    print(C.table(rows, ("", "检查项", "结果", "提示")))
    fails = sum(1 for r in rows if r[0] == "FAIL")
    warns = sum(1 for r in rows if r[0] == "WARN")
    print(f"\n{len(rows)} 项: {fails} FAIL, {warns} WARN")

    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump({"bag": args.bag, "duration_s": round(span, 2),
                       "checks": [dict(zip(("status", "name", "detail", "hint"), r))
                                  for r in rows]}, f, ensure_ascii=False, indent=2)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
