#!/usr/bin/env bash
# Continuously squeeze the active low-memory build scope.
#
# For every transient lowmem-*.scope, raise MemoryMax to:
#
#     current_cgroup_usage + MemAvailable - MIN_FREE_MB
#
# so the current compile can consume all memory that is actually free while
# leaving MIN_FREE_MB for the desktop. MemoryHigh is kept 256 MB below the
# maximum, and swap is capped separately. The build script's own watchdog
# (550 MB floor) remains the final safety net.
#
# Usage:
#   setsid ./scripts/cgroup_boost.sh &
#   echo $! > /tmp/cgroup_boost.pid
#   kill "$(cat /tmp/cgroup_boost.pid)"
set -u

MIN_FREE_MB="${MIN_FREE_MB:-700}"
MAX_CAP_MB="${MAX_CAP_MB:-4096}"
SWAP_MAX_MB="${SWAP_MAX_MB:-3072}"
INTERVAL="${INTERVAL:-2}"

CGROUP_ROOT="/sys/fs/cgroup/user.slice/user-1000.slice/user@1000.service/app.slice"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

while true; do
    avail_mb="$(awk '/^MemAvailable:/{print int($2/1024)}' /proc/meminfo)"
    for scope in "$CGROUP_ROOT"/lowmem-*.scope; do
        [ -d "$scope" ] || continue
        [ -r "$scope/memory.current" ] || continue

        cur_mb="$(awk '{print int($1/1048576)}' "$scope/memory.current" 2>/dev/null)" || continue
        target_mb=$(( cur_mb + avail_mb - MIN_FREE_MB ))
        [ "$target_mb" -gt "$MAX_CAP_MB" ] && target_mb="$MAX_CAP_MB"
        [ "$target_mb" -lt 512 ] && continue

        high_mb=$(( target_mb - 256 ))
        [ "$high_mb" -lt 256 ] && high_mb=256

        echo $(( target_mb * 1048576 )) > "$scope/memory.max" 2>/dev/null || true
        echo $(( high_mb * 1048576 )) > "$scope/memory.high" 2>/dev/null || true
        echo $(( SWAP_MAX_MB * 1048576 )) > "$scope/memory.swap.max" 2>/dev/null || true

        log "$(basename "$scope") used=${cur_mb}MB avail=${avail_mb}MB -> max=${target_mb}MB high=${high_mb}MB"
    done
    sleep "$INTERVAL"
done
