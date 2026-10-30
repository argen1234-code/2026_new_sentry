#!/usr/bin/env python3
"""Sanity-check a nav2 parameter file before you spend a run on it.

Catches the classes of mistake that cost whole runs and never raise an error at
runtime: a threshold that silently disables a feature, a parameter the plugin
never declares, two rates fighting each other, an inflation radius that covers
the entire corridor, or a source edit that a copy-install will not pick up.

    python3 lint.py                       # repo nav2_params.yaml
    python3 lint.py other.yaml --map ~/2026_new_sentry/maps/corridor_XXXX.yaml
    python3 lint.py --plant replay/plant_measured.json

Exit code 1 if any ERROR. Rules are deliberately opinionated for *this* robot;
every message carries the reason so a rule can be argued with rather than
obeyed blindly.
"""

import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_CFG = os.path.join(
    REPO, "ros_ws", "src", "sentry_navigation", "pb2025_nav_bringup",
    "config", "reality", "nav2_params.yaml")
DEFAULT_MAP = os.path.join(REPO, "maps", "corridor_XXXX.yaml")
DEFAULT_PLANT = os.path.join(REPO, "ros_ws", "tools", "replay", "plant_measured.json")

# Keys that exist in shipped configs but are never read by the plugin version in
# use. Values here are dead weight at best and misleading at worst; see
# tools/replay/variants.yaml for how each was established.
DEAD_KEYS = {
    ("planner_server.ros__parameters.GridBased", "w_heuristic_cost"):
        "Humble Theta* 未声明该参数，启发式权重固定为 1",
    ("planner_server.ros__parameters.GridBased", "how_many_corners"):
        "lib 内部固定为 8，参数无效",
    ("controller_server.ros__parameters.FollowPath", "lookahead_dist"):
        "use_velocity_scaled_lookahead_dist=true 时被忽略（由 lookahead_time 决定）",
}


def dig(d, path, default=None):
    cur = d
    for p in path.split("."):
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config", nargs="?", default=DEFAULT_CFG)
    ap.add_argument("--map", default=DEFAULT_MAP,
                    help="static map yaml, used for the corridor-width rule")
    ap.add_argument("--plant", default=DEFAULT_PLANT,
                    help="measured plant json, used for the deadband rules")
    ap.add_argument("--robot-radius", type=float, default=None,
                    help="default: read robot_radius from the costmaps")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    f = []  # (level, rule, message)

    def add(level, rule, msg):
        f.append((level, rule, msg))

    def p(path, default=None):
        return dig(cfg, path, default)

    ctl = "controller_server.ros__parameters.FollowPath."
    vs = "velocity_smoother.ros__parameters."
    lc = "local_costmap.local_costmap.ros__parameters."
    gc = "global_costmap.global_costmap.ros__parameters."

    # ---- height gates ------------------------------------------------------
    for base, name in ((lc, "local"), (gc, "global")):
        lo = p(base + "intensity_voxel_layer.min_obstacle_height")
        hi = p(base + "intensity_voxel_layer.max_obstacle_height")
        if lo is not None and hi is not None and lo > hi:
            add("ERROR", "高度带", f"{name} 体素层 min {lo} > max {hi}，永远标记不到东西")
    ptl_lo = p("pointcloud_to_laserscan.ros__parameters.min_height")
    ptl_hi = p("pointcloud_to_laserscan.ros__parameters.max_height")
    if ptl_lo is not None and ptl_hi is not None and ptl_lo > ptl_hi:
        add("ERROR", "高度带", f"pointcloud_to_laserscan min {ptl_lo} > max {ptl_hi}")
    if ptl_lo is not None and ptl_lo >= 0.16:
        add("INFO", "建图高度带",
            f"pointcloud_to_laserscan.min_height={ptl_lo}：低于它的障碍不会进 /obstacle_scan，"
            "建图看不见（导航仍可见，两条链独立）")

    # ---- mark_threshold semantics -----------------------------------------
    for base, name in ((lc, "local"), (gc, "global")):
        thr = p(base + "intensity_voxel_layer.mark_threshold")
        zres = p(base + "intensity_voxel_layer.z_resolution")
        if thr is not None and thr > 0 and zres is not None:
            need = zres * (thr + 1)
            add("WARN", "mark_threshold",
                f"{name} mark_threshold={thr}, z_resolution={zres} ⇒ 障碍竖直要展开 "
                f"≥{need:.2f} m 才标；低于此的障碍会被静默吞掉")

    # ---- controller --------------------------------------------------------
    kmin, kmax = p(ctl + "curvature_min"), p(ctl + "curvature_max")
    if kmin is not None and kmax is not None and kmin > kmax:
        add("ERROR", "曲率闸门", f"curvature_min {kmin} > curvature_max {kmax}")
    vmax, vmin = p(ctl + "v_linear_max"), p(ctl + "min_approach_linear_velocity")
    if vmax is not None and vmin is not None and vmin > vmax:
        add("ERROR", "速度范围", f"min_approach_linear_velocity {vmin} > v_linear_max {vmax}")

    # ---- plant: deadband ---------------------------------------------------
    plant = {}
    if args.plant and os.path.exists(args.plant):
        plant = json.load(open(args.plant))
    dbv = plant.get("deadband_v")
    if dbv and vmin is not None and vmin < 0.75 * dbv:
        add("WARN", "底盘死区",
            f"min_approach_linear_velocity={vmin} 低于实测死区 deadband_v={dbv:.3f} m/s，"
            "最后一段指令会被底盘吃掉（表现为终点前一顿一顿）")
    vdead = p(vs + "deadband_velocity", [0, 0, 0])
    if dbv and vdead and float(vdead[0]) == 0.0 and vmin is not None and vmin < dbv:
        add("WARN", "底盘死区",
            "velocity_smoother.deadband_velocity 为 0，不会帮忙把低于死区的指令抬起来或清零，"
            "建议 ≥ 实测死区或与 min_approach_linear_velocity 对齐")

    # ---- velocity chain ----------------------------------------------------
    smax = p(vs + "max_velocity")
    if smax and vmax is not None and float(smax[0]) < 0.8 * vmax:
        add("WARN", "速度链",
            f"velocity_smoother.max_velocity[0]={smax[0]} 明显低于 v_linear_max={vmax}，"
            "控制器以为能跑满，实际被平滑器截断")
    acc = p(vs + "max_accel")
    dec = p(vs + "max_decel")
    if acc and dec and (float(acc[0]) <= 0 or float(dec[0]) >= 0):
        add("ERROR", "速度链", "max_accel 必须为正、max_decel 必须为负")

    # ---- costmap rates -----------------------------------------------------
    for base, name in ((lc, "local"), (gc, "global")):
        upd = p(base + "update_frequency")
        pub = p(base + "publish_frequency")
        if upd is not None and pub is not None and pub > upd:
            add("WARN", "代价地图频率", f"{name} publish {pub} Hz > update {upd} Hz，会重复发同一帧")
        if upd is not None and upd < 2.0:
            add("WARN", "代价地图频率", f"{name} update_frequency={upd} Hz 偏低，动态障碍会滞后")

    # ---- inflation vs corridor --------------------------------------------
    robot_r = args.robot_radius
    if robot_r is None:
        robot_r = p(gc + "robot_radius") or p(lc + "robot_radius")
    infl = p(gc + "inflation_layer.inflation_radius")
    if robot_r is not None and infl is not None and infl < robot_r:
        add("ERROR", "膨胀半径", f"inflation_radius={infl} < robot_radius={robot_r}，会撞")
    if infl is not None and args.map and os.path.exists(args.map):
        try:
            import common as C
            grid, res, ox, oy = C.load_map(args.map)
            # A corridor that runs along x is only as wide as its free run in y,
            # so sweep x and take the runs that contain y=0. The median is the
            # corridor; the minimum is a passage (or a stray free speck, so it
            # is printed for inspection rather than used for the rule).
            widths = []
            narrow = None
            x = ox
            while x < ox + grid.shape[1] * res:
                run = C.free_run(grid, res, ox, oy, x, 0.0, axis="y")
                if run:
                    widths.append(run[0])
                    if narrow is None or run[0] < narrow[0]:
                        narrow = (run[0], run[1], run[2], x)
                x += 0.10
            if widths:
                import statistics
                # Runs under 0.6 m are stray free specks, runs over 3 m are open
                # rooms; neither is "the passage the robot has to squeeze through".
                credible = [w for w in widths if 0.6 <= w <= 3.0] or widths
                credible.sort()
                passage = credible[int(0.10 * (len(credible) - 1))]
                typical = statistics.median(widths)
                add("INFO", "通道净宽",
                    f"含 y=0 的横向通道：中位 {typical:.2f} m，窄侧 10 分位 {passage:.2f} m，"
                    f"最窄 {narrow[0]:.2f} m（y {narrow[1]:.2f}..{narrow[2]:.2f} @ x={narrow[3]:.2f}）")
                if infl is not None and infl >= 0.5 * passage:
                    add("WARN", "膨胀半径",
                        f"inflation_radius={infl} ≥ 窄侧通道半宽 {passage / 2:.2f} m ⇒ "
                        "整条通道带代价，规划器的路径由代价场微差决定，会左右摇摆"
                        "（实测蛇形横偏中位 0.52 m）")
        except Exception as exc:  # a linter must not die on a bad map
            add("INFO", "通道净宽", f"读地图失败，跳过：{exc}")

    # ---- dead keys ---------------------------------------------------------
    for (prefix, key), why in DEAD_KEYS.items():
        val = dig(cfg, prefix + "." + key)
        if val is not None:
            add("WARN", "死参数", f"{prefix}.{key}={val} 不起作用：{why}")

    # ---- behaviour tree wiring --------------------------------------------
    bt = p("bt_navigator.ros__parameters.default_nav_to_pose_bt_xml")
    bt_text = None
    if bt:
        # $(find-pkg-share X)/rel -> resolve through the install tree
        resolved = bt
        if bt.startswith("$(find-pkg-share "):
            pkg, rest = bt[len("$(find-pkg-share "):].split(")", 1)
            resolved = os.path.join(REPO, "ros_ws", "install", pkg, "share", pkg,
                                    rest.lstrip("/"))
        if not os.path.exists(resolved):
            add("ERROR", "行为树", f"BT 文件不存在：{resolved}")
        else:
            bt_text = open(resolved).read()
            add("INFO", "行为树", f"实际 BT：{resolved}")
    smoothers = p("smoother_server.ros__parameters.smoother_plugins")
    if smoothers and bt_text is not None:
        if "SmoothPath" not in bt_text:
            add("INFO", "行为树",
                f"smoother_server 配了 {smoothers} 但 BT 里没有 SmoothPath ⇒ 平滑器是死配置")
        else:
            added = list(smoothers)
            if "simple_smoother" not in added and "simple_smoother" in bt_text:
                add("ERROR", "行为树", f"BT 用 simple_smoother，但 smoother 插件只有 {added}")

    # ---- install freshness -------------------------------------------------
    pkg_src = os.path.join(REPO, "ros_ws", "src", "sentry_navigation",
                           "pb2025_nav_bringup")
    pkg_inst = os.path.join(REPO, "ros_ws", "install", "pb2025_nav_bringup")
    inst_cfg = os.path.join(pkg_inst, "share", "pb2025_nav_bringup", "config",
                            "reality", "nav2_params.yaml")
    if os.path.exists(inst_cfg):
        if os.path.realpath(inst_cfg) != os.path.realpath(args.config):
            add("WARN", "安装新鲜度",
                f"install 里的参数不是源码的软链：{inst_cfg} -> {os.path.realpath(inst_cfg)}；"
                "改源码后需要 colcon build 才会生效")
        else:
            add("INFO", "安装新鲜度", "symlink-install：改源码即生效")

    # ---- report ------------------------------------------------------------
    order = {"ERROR": 0, "WARN": 1, "INFO": 2}
    f.sort(key=lambda r: order[r[0]])
    print(f"=== {args.config}")
    for level, rule, msg in f:
        print(f"[{level:5s}] {rule}: {msg}")
    n_err = sum(1 for r in f if r[0] == "ERROR")
    n_warn = sum(1 for r in f if r[0] == "WARN")
    print(f"\n{len(f)} 条: {n_err} ERROR, {n_warn} WARN")
    return 1 if n_err else 0


if __name__ == "__main__":
    sys.exit(main())
