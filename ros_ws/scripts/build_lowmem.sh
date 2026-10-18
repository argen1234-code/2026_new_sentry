#!/usr/bin/env bash
# Low-memory, resumable colcon build for memory-constrained Jetson boards.
#
# Guarantees:
#   * one package at a time, one compiler job at a time (-j1);
#   * every package runs inside its own systemd --user scope with
#     MemoryMax / MemoryHigh / MemorySwapMax limits, so an oversized
#     compiler is OOM-killed instead of freezing the whole board;
#   * a watchdog samples MemAvailable every 2 s and stops the active
#     scope if free memory falls below MEM_FLOOR_MB;
#   * progress is resumable with RESUME_FROM=<package>.
#
# Usage:
#   ./scripts/build_lowmem.sh                 # full workspace
#   ONLY=livox_ros_driver2 ./scripts/build_lowmem.sh
#   RESUME_FROM=point_lio ./scripts/build_lowmem.sh
#   DRY_RUN=1 ./scripts/build_lowmem.sh       # print order only
#
# Tunables (environment):
#   JOBS=1                compiler jobs inside a package
#   MEM_FLOOR_MB=450      abort/kill if MemAvailable drops below this
#   MIN_START_MB=700      refuse to start a package below this
#   MEM_HIGH_PCT=55       MemoryHigh as % of MemAvailable (throttle)
#   MEM_MAX_PCT=75        MemoryMax as % of MemAvailable (hard kill)
#   MAX_CAP_MB=2304       absolute per-package MemoryMax cap
#   SWAP_MAX_MB=1024      swap a package may use (0 disables swap)
#   CPU_QUOTA=150%        cgroup CPU quota for the build scope
#   SKIP_PKGS="..."       packages to skip (default: standard_robot_pp_ros2)
#   PKG_MAX_MB_<pkg>=N    per-package MemoryMax override, e.g.
#                         PKG_MAX_MB_point_lio=2048
#
set -euo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
JOBS="${JOBS:-1}"
MEM_FLOOR_MB="${MEM_FLOOR_MB:-450}"
MIN_START_MB="${MIN_START_MB:-700}"
MEM_HIGH_PCT="${MEM_HIGH_PCT:-55}"
MEM_MAX_PCT="${MEM_MAX_PCT:-75}"
MAX_CAP_MB="${MAX_CAP_MB:-2304}"
SWAP_MAX_MB="${SWAP_MAX_MB:-1024}"
CPU_QUOTA="${CPU_QUOTA:-150%}"
SKIP_PKGS="${SKIP_PKGS:-none}"
RESUME_FROM="${RESUME_FROM:-}"
ONLY="${ONLY:-}"
DRY_RUN="${DRY_RUN:-0}"

if [ ! -f "$ROS_SETUP" ]; then
    printf 'ROS setup not found: %s\n' "$ROS_SETUP" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS/log/lowmem"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/build_$STAMP.log"
CSV="$LOG_DIR/mem_$STAMP.csv"
STATE_DIR="$(mktemp -d)"
UNIT_FILE="$STATE_DIR/unit"
PKG_FILE="$STATE_DIR/pkg"
: > "$UNIT_FILE"
: > "$PKG_FILE"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }

mem_avail_mb() { awk '/^MemAvailable:/{print int($2/1024)}' /proc/meminfo; }
mem_used_mb() { awk '/^MemTotal:/{t=$2}/^MemAvailable:/{a=$2}END{print int((t-a)/1024)}' /proc/meminfo; }
swap_used_mb() { awk '/^SwapTotal:/{t=$2}/^SwapFree:/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo; }
swap_total_mb() { awk '/^SwapTotal:/{print int($2/1024)}' /proc/meminfo; }
load1() { awk '{print $1}' /proc/loadavg; }

ros_pkg_present() {
    [ -d "/opt/ros/${ROS_DISTRO:-jazzy}/share/$1" ] ||
    [ -d "$WS/install/$1/share/$1" ]
}

source_ws() {
    if [ -f "$WS/install/local_setup.bash" ]; then
        set +u
        # shellcheck disable=SC1091
        source "$WS/install/local_setup.bash"
        set -u
    fi
}

# ---------------------------------------------------------------- watchdog
WATCHDOG_PID=""
start_watchdog() {
    printf 'ts,mem_available_mb,mem_used_mb,swap_used_mb,load1,package\n' > "$CSV"
    (
        while kill -0 "$$" 2>/dev/null; do
            av="$(mem_avail_mb)"
            printf '%s,%s,%s,%s,%s,%s\n' \
                "$(date +%s)" "$av" "$(mem_used_mb)" "$(swap_used_mb)" \
                "$(load1)" "$(cat "$PKG_FILE" 2>/dev/null || true)" >> "$CSV"
            unit="$(cat "$UNIT_FILE" 2>/dev/null || true)"
            if [ -n "$unit" ] && [ "$av" -lt "$MEM_FLOOR_MB" ]; then
                printf '[watchdog] MemAvailable=%sMB < floor=%sMB; stopping %s.scope\n' \
                    "$av" "$MEM_FLOOR_MB" "$unit" >> "$LOG"
                systemctl --user stop "$unit.scope" 2>/dev/null || true
            fi
            sleep 2
        done
    ) &
    WATCHDOG_PID=$!
}
stop_watchdog() {
    if [ -n "$WATCHDOG_PID" ]; then
        kill "$WATCHDOG_PID" 2>/dev/null || true
        wait "$WATCHDOG_PID" 2>/dev/null || true
    fi
    rm -rf "$STATE_DIR"
}
trap stop_watchdog EXIT

# ------------------------------------------------------------ build order
build_order() {
    python3 - "$WS" <<'PY'
import glob
import os
import sys
import xml.etree.ElementTree as ET

root = sys.argv[1]
pkgs = {}
for xml_path in glob.glob(os.path.join(root, "src", "**", "package.xml"), recursive=True):
    try:
        node = ET.parse(xml_path).getroot()
    except ET.ParseError:
        continue
    name = (node.findtext("name") or "").strip()
    if not name:
        continue
    deps = set()
    for tag in ("depend", "build_depend", "buildtool_depend",
                "build_export_depend", "exec_depend"):
        for elem in node.findall(tag):
            if elem.text:
                deps.add(elem.text.strip())
    pkgs[name] = deps

names = set(pkgs)
indegree = {n: len(pkgs[n] & names) for n in names}
reverse = {n: set() for n in names}
for n in names:
    for dep in pkgs[n] & names:
        reverse[dep].add(n)

ready = sorted(n for n in names if indegree[n] == 0)
order = []
while ready:
    n = ready.pop(0)
    order.append(n)
    for m in sorted(reverse[n]):
        indegree[m] -= 1
        if indegree[m] == 0:
            ready.append(m)
            ready.sort()
# A dependency cycle would be a repo bug; emit the rest deterministically.
order.extend(sorted(names - set(order)))
print("\n".join(order))
PY
}

missing_build_deps() {
    case "$1" in
        point_lio|terrain_analysis|terrain_analysis_ext)
            ros_pkg_present pcl_ros || echo "pcl_ros" ;;
        standard_robot_pp_ros2)
            local missing=""
            ros_pkg_present serial_driver || missing+="serial_driver "
            ros_pkg_present asio_cmake_module || missing+="asio_cmake_module "
            echo "$missing" ;;
    esac
}

# ------------------------------------------------------------------- main
ORDER="$(build_order)"
if [ -n "$ONLY" ]; then
    ORDER="$ONLY"
fi

log "workspace: $WS"
log "ROS: ${ROS_DISTRO:-unknown}  jobs=$JOBS  log=$LOG  mem_csv=$CSV"
log "available=$(( $(mem_avail_mb) ))MB  swap_total=$(swap_total_mb)MB  load=$(load1)"

if [ "$(swap_total_mb)" -eq 0 ]; then
    log "WARNING: no swap. A heavy translation unit can still pin memory;"
    log "         run ./scripts/setup_swap.sh with sudo before point_lio."
fi
if [ "$(mem_avail_mb)" -lt "$MIN_START_MB" ]; then
    log "ABORT: only $(( $(mem_avail_mb) ))MB available (< ${MIN_START_MB}MB)."
    log "       close Firefox/extra VSCode windows, or enable swap first."
    exit 2
fi
if [ "$DRY_RUN" = "1" ]; then
    log "dry run: build order"
    printf '%s\n' "$ORDER"
    exit 0
fi

start_watchdog
source_ws

built=0
failed=0
skipped=0
reached_resume=0
[ -z "$RESUME_FROM" ] && reached_resume=1

for pkg in $ORDER; do
    if [ "$reached_resume" -eq 0 ]; then
        if [ "$pkg" = "$RESUME_FROM" ]; then
            reached_resume=1
        else
            log "skip (before RESUME_FROM): $pkg"
            continue
        fi
    fi

    case " $SKIP_PKGS " in
        *" $pkg "*)
            log "skip (SKIP_PKGS): $pkg"
            skipped=$((skipped + 1))
            continue ;;
    esac

    deps_missing="$(missing_build_deps "$pkg")"
    if [ -n "$deps_missing" ]; then
        log "skip (missing deps: $deps_missing): $pkg"
        skipped=$((skipped + 1))
        continue
    fi

    av="$(mem_avail_mb)"
    if [ "$av" -lt "$MIN_START_MB" ]; then
        log "ABORT before $pkg: available=${av}MB < ${MIN_START_MB}MB"
        log "       free memory or add swap, then rerun with RESUME_FROM=$pkg"
        exit 3
    fi

    cap_var="PKG_MAX_MB_${pkg}"
    cap_override="${!cap_var:-0}"
    max=$(( av * MEM_MAX_PCT / 100 ))
    [ "$max" -gt "$MAX_CAP_MB" ] && max="$MAX_CAP_MB"
    [ "$cap_override" -gt 0 ] && max="$cap_override"
    high=$(( av * MEM_HIGH_PCT / 100 ))
    [ "$high" -ge "$max" ] && high=$(( max - 256 ))
    [ "$high" -lt 256 ] && high=256
    if [ "$max" -le 256 ]; then
        log "ABORT before $pkg: calculated MemoryMax=${max}MB is too small"
        exit 3
    fi

    swap_cap="$SWAP_MAX_MB"
    [ "$(swap_total_mb)" -eq 0 ] && swap_cap=0

    unit="lowmem-${pkg//[^A-Za-z0-9-]/-}-$$"
    printf '%s' "$unit" > "$UNIT_FILE"
    printf '%s' "$pkg" > "$PKG_FILE"

    log "build $pkg (avail=${av}MB high=${high}M max=${max}M swap=${swap_cap}M)"
    set +e
    systemd-run --user --scope --collect --quiet --unit="$unit" \
        -p "MemoryHigh=${high}M" -p "MemoryMax=${max}M" \
        -p "MemorySwapMax=${swap_cap}M" -p "CPUQuota=${CPU_QUOTA}" -p "IOWeight=50" \
        nice -n 10 ionice -c 2 -n 6 env \
        MAKEFLAGS="-j${JOBS}" CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" \
        colcon build --packages-select "$pkg" --parallel-workers 1 \
        --symlink-install --event-handlers console_cohesion+ \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_FLAGS_RELEASE="-O2 -g0 -DNDEBUG" \
        -DCMAKE_CXX_FLAGS_RELEASE="-O2 -g0 -DNDEBUG" \
        >> "$LOG" 2>&1
    rc=$?
    set -e
    : > "$UNIT_FILE"
    : > "$PKG_FILE"

    if [ "$rc" -eq 0 ]; then
        built=$((built + 1))
        log "ok: $pkg"
        source_ws
    else
        failed=$((failed + 1))
        log "FAILED rc=$rc: $pkg"
        tail -n 40 "$LOG" | sed 's/^/[log] /' >&2
        if [ "$rc" -eq 137 ] || [ "$rc" -eq 143 ]; then
            log "       package was killed by the memory guard or by SIGTERM."
            log "       enable swap / free memory, then: RESUME_FROM=$pkg $0"
        fi
        exit "$rc"
    fi
done

log "done: built=$built skipped=$skipped failed=$failed"
log "source the workspace with: source $WS/install/local_setup.bash"
