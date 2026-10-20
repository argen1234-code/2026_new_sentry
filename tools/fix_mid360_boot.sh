#!/usr/bin/env bash
# Fix the boot delay/hang that appears when the Livox MID360 is connected.
#
# Root cause:
#   The normal "Ubuntu L4T" UEFI entry fails on this board and the firmware
#   falls through to the auto-created network boot entries (PXEv4/PXEv6/
#   HTTPv4/HTTPv6). With the MID360 plugged in, enP8p1s0 has carrier, so those
#   entries wait for a DHCP server that does not exist and the boot appears to
#   hang. With no lidar the same entries fail immediately and the firmware
#   reaches the UEFI Shell, which runs startup.nsh and boots the kernel.
#
# This script only changes the UEFI BootOrder: the UEFI Shell is placed before
# the network entries, so startup.nsh runs immediately. The firmware entries
# themselves are not deleted and the existing startup.nsh fallback is not
# touched.
#
# Usage:
#   sudo ./tools/fix_mid360_boot.sh              # drop network entries
#   sudo ./tools/fix_mid360_boot.sh --keep-network
#   ./tools/fix_mid360_boot.sh --dry-run         # print only
#
set -euo pipefail

KEEP_NETWORK=0
DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --keep-network) KEEP_NETWORK=1 ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help)
            sed -n '2,22p' "$0"
            exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

if [ "$DRY_RUN" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0 $*" >&2
    exit 1
fi

entries="$(efibootmgr)"
have() { printf '%s\n' "$entries" | grep -q "^Boot$1\*"; }

# Ubuntu L4T -> UEFI Shell -> SD -> Setup -> BootManagerMenuApp.
# The shell is deliberately before the SD/network fallbacks: it is the known
# working path on this board and runs /startup.nsh.
order=""
for id in 0009 0007 0001 0000 0006; do
    if have "$id"; then
        order="${order:+$order,}$id"
    fi
done
if [ "$KEEP_NETWORK" -eq 1 ]; then
    for id in 0004 0003 0002 0005; do
        if have "$id"; then
            order="${order:+$order,}$id"
        fi
    done
fi

if [ -z "$order" ]; then
    echo "error: none of the expected boot entries exist" >&2
    exit 1
fi

echo "current:"
printf '%s\n' "$entries"
echo
echo "new BootOrder: $order"

if [ "$DRY_RUN" -eq 1 ]; then
    echo "dry run: not writing EFI variables"
    exit 0
fi

backup_dir="/home/argen/boot-fix-backup/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
efibootmgr -v > "$backup_dir/efibootmgr-before.txt"
printf '%s\n' "$order" > "$backup_dir/BootOrder.new"
echo "backup: $backup_dir/efibootmgr-before.txt"

efibootmgr -o "$order"
echo
echo "result:"
efibootmgr

cat <<'EOF'

Done.
  * Network PXE/HTTP entries are omitted from the boot order (unless
    --keep-network was used), so a connected MID360 can no longer make the
    firmware sit in a DHCP timeout.
  * The UEFI Shell still runs /startup.nsh as the fallback boot path.

If the normal Ubuntu L4T entry should boot without the UEFI Shell, enter UEFI
setup at the next boot (ESC) and set:
    Device Manager -> NVIDIA Configuration -> L4T Configuration
      OS chain A status : Normal
      L4T Boot Mode     : ExtLinux
    Save & Exit

Roll back the boot order with:
    sudo efibootmgr -o 0009,0001,0004,0003,0002,0005,0000,0006,0007
EOF
