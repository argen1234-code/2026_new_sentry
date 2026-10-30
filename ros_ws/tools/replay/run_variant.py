#!/usr/bin/env python3
"""Run one parameter variant through the bag-replay harness and score it.

    python3 run_variant.py --variant G1 [--plant ideal] [--no-crop]

Sequence: build a params copy -> start the offline stack -> advance /clock ahead
of the bag so the lifecycle stack finishes autostarting -> play only the world
clouds -> drive the scenario's goals on bag time -> record -> score.

The real nav2_params.yaml is only ever read.
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
NAV_PARAMS = os.path.join(
    REPO, "ros_ws", "src", "sentry_navigation", "pb2025_nav_bringup",
    "config", "reality", "nav2_params.yaml")

LIFECYCLE_NODES = [
    "map_server", "controller_server", "smoother_server", "planner_server",
    "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
]

RECORD_TOPICS = [
    "/plan", "/cmd_vel", "/cmd_vel_controller", "/cmd_vel_nav2_result",
    "/odometry", "/local_costmap/costmap", "/global_costmap/costmap", "/tf",
    # /tf_static carries the bench's map->odom; without it a replay bag cannot
    # be checked for "robot inside a lethal cell" (tools/nav_tune/doctor.py).
    "/tf_static",
]


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# --------------------------------------------------------------- setup

def load_defs(path):
    with open(path) as f:
        return yaml.safe_load(f)["groups"]


def build_params(changes, out_path):
    with open(NAV_PARAMS) as f:
        text = f.read().replace("<robot_namespace>", "")
    data = yaml.safe_load(text)
    for dotted, value in changes.items():
        tokens = dotted.split(".")
        cur = data
        for tok in tokens[:-1]:
            if tok not in cur:
                raise KeyError(f"variant path not found: {dotted} (at {tok})")
            cur = cur[tok]
        if tokens[-1] not in cur:
            log(f"WARNING: {dotted} not present in baseline, adding it")
        if isinstance(value, str) and "{HERE}" in value:
            value = value.replace("{HERE}", HERE)
        cur[tokens[-1]] = value
    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out_path


def bag_start_time(bag):
    with open(os.path.join(bag, "metadata.yaml")) as f:
        meta = yaml.safe_load(f)["rosbag2_bagfile_information"]
    start = meta["starting_time"]
    key = "nanoseconds_since_epoch" if "nanoseconds_since_epoch" in start else "nanoseconds"
    return int(start[key]) * 1e-9


def stale_replay():
    r = subprocess.run(["pgrep", "-af", "replay.launch.py|tools/replay/(kinematic_sim|cloud_crop)"],
                       capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if "pgrep" not in ln and "run_variant" not in ln]


def real_stack_running():
    """Anything that would forward /cmd_vel to the chassis must be dead first."""
    r = subprocess.run(["pgrep", "-af", "usb_bridge|cmd_vel_to_unified"],
                       capture_output=True, text=True)
    lines = [ln for ln in r.stdout.splitlines()
             if "pgrep" not in ln and "run_variant" not in ln]
    return lines


def plant_path(mode):
    if mode == "measured":
        return os.path.join(HERE, "plant_measured.json")
    ideal = {
        "v_gain": 1.0, "wz_gain": 1.0, "deadband_v": 0.0, "deadband_w": 0.0,
        "frame_yaw_deg": -45.0, "lag_tau": 0.0, "latency": 0.0,
    }
    path = os.path.join(HERE, "plant_ideal.json")
    with open(path, "w") as f:
        json.dump(ideal, f, indent=2)
    return path


def spawn(cmd, log_path):
    f = open(log_path, "a")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)


def stop(p, name, sig=signal.SIGINT, wait=10.0):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), sig)
    except ProcessLookupError:
        return
    t_end = time.monotonic() + wait
    while time.monotonic() < t_end:
        if p.poll() is not None:
            return
        time.sleep(0.2)
    log(f"{name} did not exit on {sig.name}, killing")
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


LEFTOVERS = [
    "kinematic_sim.py", "cloud_crop.py", "goal_schedule.py", "fake_vel_transform",
    "map_server", "planner_server", "controller_server", "bt_navigator",
    "behavior_server", "waypoint_follower", "smoother_server", "velocity_smoother",
]


def wait_for_quiet(timeout=15.0):
    """Wait until no harness node is left behind.

    A leaked map_server keeps the service name occupied, and the next run's
    lifecycle manager then sends CONFIGURE into the void: map_server stays
    inactive, the static layer complains "no map received", and the run dies
    with 'lifecycle never active'. Seen when two runs are started back to back.
    """
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        out = subprocess.run(["pgrep", "-af", "|".join(LEFTOVERS)],
                             capture_output=True, text=True).stdout.strip()
        if not out:
            return True
        time.sleep(0.5)
    log("WARNING: nodes still running after teardown:\n  " +
        "\n  ".join(out.splitlines()[:8]))
    return False


def lifecycle_all_active(timeout, log_path):
    """Ask every node for its state from one process (see wait_active.py)."""
    try:
        r = subprocess.run(
            ["python3", os.path.join(HERE, "wait_active.py"), "--ros-args",
             "-p", "nodes:=" + ",".join(LIFECYCLE_NODES),
             "-p", f"timeout:={timeout}"],
            capture_output=True, text=True, timeout=timeout + 30)
    except subprocess.TimeoutExpired:
        return False, ["wait_active timed out"]
    with open(log_path, "w") as f:
        f.write(r.stdout + r.stderr)
    if r.returncode == 0:
        return True, []
    missing = (r.stdout or r.stderr).strip().splitlines()
    return False, [ln for ln in missing if ln.strip()][-1:] or ["unknown"]


def manual_wait(goals, log_path, budget):
    """Let the operator drive goals from RViz; announce the preemption moment."""
    g1, g2 = goals[0], goals[-1]
    banner = [
        "",
        "=" * 68,
        "回放已开始。请在 RViz 里用【Nav2 Goal】工具点目标：",
        f"  1) 马上：x={g1['x']:.2f}  y={g1['y']:.2f}   （录包里的第一个目标）",
        f"  2) 到 t={g2['t']:.1f}s 时改点：x={g2['x']:.2f}  y={g2['y']:.2f}",
        "  下面会到点自动提醒。结束请在本终端按 Ctrl-C。",
        "=" * 68,
        "",
    ]
    for line in banner:
        log(line)
    t_play = time.monotonic()
    marks = [(5.0, "如果还没设目标，现在设 1)"), (g2["t"], "★ 现在改点 2)"),
             (g2["t"] + 60.0, "回放接近尾段"), (g2["t"] + 75.0, "可以按 Ctrl-C 结束了")]
    done = set()
    with open(log_path, "w") as f:
        f.write("\n".join(banner) + "\n")
        try:
            while True:
                el = time.monotonic() - t_play
                for t_mark, msg in marks:
                    if el >= t_mark and t_mark not in done:
                        done.add(t_mark)
                        log(f"[t={el:5.1f}s] {msg}")
                        f.write(f"[t={el:5.1f}s] {msg}\n")
                        f.flush()
                if el > budget:
                    log("manual budget exhausted, stopping")
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            log("Ctrl-C: stopping replay")


# --------------------------------------------------------------- run

def run_one(name, variant, args, out_root):
    stale = stale_replay()
    if stale and not args.force:
        raise SystemExit("a replay stack is still running:\n  " + "\n  ".join(stale)
                         + "\nstop it (or pass --force) before starting another run")
    run_id = f"{name}__{args.plant}" + ("" if args.crop else "__nocrop")
    if args.tag:
        run_id += f"__{args.tag}"
    run_dir = os.path.join(out_root, run_id)
    if os.path.exists(run_dir):
        if not args.force:
            raise SystemExit(f"{run_dir} exists; pass --force to replace it")
        shutil.rmtree(run_dir)
    os.makedirs(run_dir)
    tag = lambda n: os.path.join(run_dir, n)

    params = build_params(variant.get("changes", {}), tag("params.yaml"))
    plant = plant_path(args.plant)
    bag_start = bag_start_time(args.bag)
    with open(tag("meta.json"), "w") as f:
        json.dump({"variant": name, "label": variant.get("label"),
                   "changes": variant.get("changes", {}), "plant": args.plant,
                   "crop": args.crop, "bag": args.bag, "bag_start": bag_start,
                   "map_odom": {k: getattr(args, f"map_odom_{k}")
                                for k in ("x", "y", "z", "yaw")}}, f, indent=2)

    log(f"=== {run_id}: {variant.get('label','')}")
    launch_args = [
        "ros2", "launch", os.path.join(HERE, "replay.launch.py"),
        f"params_file:={params}",
        f"plant_json:={plant}",
        f"crop:={'true' if args.crop else 'false'}",
        f"map:={args.map}",
        f"rviz:={'true' if (args.rviz or args.manual) else 'false'}",
    ]
    for key in ("x", "y", "z", "yaw"):
        val = getattr(args, f"map_odom_{key}", None)
        if val is not None:
            launch_args.append(f"map_odom_{key}:={val}")
    launch = spawn(launch_args, tag("launch.log"))

    preroll = spawn([
        "python3", os.path.join(HERE, "clock_preroll.py"), "--ros-args",
        "-p", f"start_sec:={bag_start - 60.0:.3f}",
        "-p", f"end_sec:={bag_start - 2.0:.3f}",
        "-p", "rate:=10.0",
    ], tag("preroll.log"))

    rec = play = None
    try:
        ok, missing = lifecycle_all_active(args.lifecycle_timeout, tag("lifecycle.log"))
        stop(preroll, "clock_preroll")
        if not ok:
            tail = "".join(open(tag("launch.log")).readlines()[-40:])
            raise SystemExit(
                f"lifecycle never active, missing={missing}\n--- launch tail ---\n{tail}")
        log("lifecycle active; starting playback")

        rec = spawn(["ros2", "bag", "record", "-o", tag("bag")] + args.record_topics,
                    tag("record.log"))
        time.sleep(1.5)

        # No `--ros-args -r` here: ros2 bag play's argparse reads `-r` as the
        # abbreviation of --rate and dies with "is not the valid type (float)".
        play = spawn([
            "ros2", "bag", "play", args.bag,
            "--clock", "--rate", str(args.rate),
            "--topics", "/terrain_map", "/terrain_map_ext",
            "--remap", "/terrain_map:=/terrain_map_raw",
            "/terrain_map_ext:=/terrain_map_ext_raw",
        ], tag("play.log"))

        goals = variant.get("goals") or args.goals
        if args.manual:
            manual_wait(goals, tag("manual.log"), args.goal_timeout)
        else:
            goals_file = tag("goals.json")
            with open(goals_file, "w") as f:
                json.dump(goals, f, indent=2)
            t_goal0 = time.monotonic()
            sch = subprocess.run([
                "python3", os.path.join(HERE, "goal_schedule.py"), "--ros-args",
                "-p", f"goals_file:={goals_file}",
                "-p", f"t_origin:={bag_start:.4f}",
                "-p", "use_sim_time:=true",
                "-p", f"result_timeout:={args.goal_timeout}",
            ], capture_output=True, text=True, timeout=args.goal_timeout + 180)

            log(f"goal_schedule finished in {time.monotonic() - t_goal0:.1f}s rc={sch.returncode}")
            with open(tag("goals.log"), "w") as f:
                f.write(sch.stdout + "\n--- stderr ---\n" + sch.stderr)
    finally:
        # Always tear down in reverse start order; a leaked nav2 stack would keep
        # holding /cmd_vel and TF and poison the next run.
        time.sleep(1.0)
        stop(play, "bag play", wait=10.0)
        stop(rec, "bag record", wait=15.0)
        stop(launch, "launch", wait=20.0)
        wait_for_quiet()
    time.sleep(1.0)

    log("scoring")
    r = subprocess.run(
        ["python3", os.path.join(HERE, "metrics.py"), tag("bag"),
         "--json", tag("metrics.json")],
        capture_output=True, text=True)
    if r.returncode != 0:
        log("metrics failed:\n" + r.stderr[-2000:])
        return None
    with open(tag("metrics.json")) as f:
        res = json.load(f)
    res["variant"] = name
    res["label"] = variant.get("label", "")
    res["plant"] = args.plant
    res["crop"] = args.crop
    with open(tag("metrics.json"), "w") as f:
        json.dump(res, f, indent=2)
    log(f"=== {run_id}: {res.get('duration_s')}s  {res.get('path_length_m')}m  "
        f"{res.get('replan_hz')}Hz  jitter p90 {res.get('jitter_p90_m')}")
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", required=True)
    ap.add_argument("--scenario", default="scenario_a.json",
                    help="scenario json in this directory (or an absolute path)")
    ap.add_argument("--defs", default=os.path.join(HERE, "variants.yaml"))
    ap.add_argument("--bag", default=None)
    ap.add_argument("--plant", default="measured", choices=["measured", "ideal"])
    ap.add_argument("--no-crop", dest="crop", action="store_false")
    ap.add_argument("--map", default=os.path.join(REPO, "maps", "corridor_1025_tape.yaml"))
    ap.add_argument("--out", default="/tmp/replay/results")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--goal-timeout", type=float, default=200.0)
    ap.add_argument("--lifecycle-timeout", type=float, default=90.0)
    ap.add_argument("--tag", default="")
    ap.add_argument("--map-odom-z", type=float, default=None,
                    help="override map_odom_z (the value decides whether the floor "
                         "sits above or below the voxel layer's min_obstacle_height)")
    ap.add_argument("--map-odom-x", type=float, default=None,
                    help="override map_odom_x; recorded runs keep a constant offset, "
                         "copy it from the bag's map->odom")
    ap.add_argument("--map-odom-y", type=float, default=None)
    ap.add_argument("--map-odom-yaw", type=float, default=None)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--manual", action="store_true",
                    help="skip goal_schedule; drive goals from RViz and Ctrl-C to finish")
    ap.add_argument("--rviz", action="store_true", help="start RViz with the Nav2 compact view")
    ap.add_argument("--record-topics", nargs="*", default=RECORD_TOPICS)
    args = ap.parse_args()

    running = real_stack_running()
    if running:
        raise SystemExit(
            "refusing to replay: the real chassis bridge is still running and would "
            "forward this replay's /cmd_vel to the robot:\n  " + "\n  ".join(running) +
            "\nrun ros_ws/tools/stop_ros_stack.sh first")

    scenario_path = args.scenario if os.path.isabs(args.scenario) \
        else os.path.join(HERE, args.scenario)
    scenario = json.load(open(scenario_path))
    if args.bag is None:
        args.bag = scenario["bag"]
    args.goals = scenario["goals"]

    groups = load_defs(args.defs)
    if args.variant not in groups:
        raise SystemExit(f"unknown variant {args.variant}; have {sorted(groups)}")
    os.makedirs(args.out, exist_ok=True)
    run_one(args.variant, groups[args.variant], args, args.out)


if __name__ == "__main__":
    main()
