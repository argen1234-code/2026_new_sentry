# MID360 插入时开机卡住的修复

## 现象

- 插着 Livox MID360 开机时，系统长时间停在开机画面。
- 拔掉雷达后可以进入系统，但会落到 UEFI Interactive Shell。
- 当前系统实际是通过 `/startup.nsh` 在 UEFI Shell 中手动引导内核。

## 根因

这台 Jetson Orin Nano Super 从 microSD 启动，UEFI 里的正常启动项
`Boot0009 Ubuntu L4T` 无法完成引导（此前排查怀疑 QSPI 中
`OS chain A status = Unbootable`）。固件按 `BootOrder` 依次尝试：

```text
0009 Ubuntu L4T       失败
0001 UEFI SD Device   失败
0004 UEFI HTTPv4      ← 网络启动
0003 UEFI PXEv6       ← 网络启动
0002 UEFI PXEv4       ← 网络启动
0005 UEFI HTTPv6      ← 网络启动
0000 Enter Setup
0006 BootManagerMenuApp
0007 UEFI Shell       成功，执行 /startup.nsh
```

MID360 插在 `enP8p1s0` 上时，该网口有载波。固件进入 PXE/HTTP 启动项后
会等待 DHCP 服务器，而 MID360 不提供 DHCP，因此超时时间很长，表现为
“卡在开机页面”。拔掉雷达时这些网络启动项立即失败，所以能较快落到
UEFI Shell。

`nv-l4t-bootloader-config.service` 也会失败，因为它通过 `BootCurrent`
判断启动介质；当前 `BootCurrent=0007`（UEFI Shell），它无法识别
“已知启动介质”，所以不会挂载 ESP。这不是卡死的原因，而是同一故障链的
下游现象。

## 立即修复（不改变现有启动方式）

把 UEFI Shell 放到网络启动项之前，并移除网络启动项：

```bash
sudo /home/argen/2026_new_sentry/tools/fix_mid360_boot.sh
```

脚本只修改 UEFI `BootOrder`，不删除启动项、不改 `/startup.nsh`、不改
分区或内核。新的顺序为：

```text
0009 Ubuntu L4T
0007 UEFI Shell
0001 UEFI SD Device
0000 Enter Setup
0006 BootManagerMenuApp
```

效果：

- 插着 MID360 开机时，固件不会再进入 PXE/HTTP 的 DHCP 等待。
- 如果 `Boot0009` 仍然失败，会立即进入 UEFI Shell 并执行
  `/startup.nsh`，保持当前可用的启动路径。
- 如果以后 `Boot0009` 修好，它会优先正常启动。

先预览、不写入：

```bash
/home/argen/2026_new_sentry/tools/fix_mid360_boot.sh --dry-run
```

保留网络启动项、只把它们排到最后：

```bash
sudo /home/argen/2026_new_sentry/tools/fix_mid360_boot.sh --keep-network
```

回滚：

```bash
sudo efibootmgr -o 0009,0001,0004,0003,0002,0005,0000,0006,0007
```

## 彻底修复正常启动（可选，需在机器前操作）

要让系统不再依赖 `startup.nsh`，需要在 UEFI 菜单里恢复 L4T 启动状态：

1. 开机按 `ESC` 进入 UEFI Setup。
2. `Device Manager` → `NVIDIA Configuration` → `L4T Configuration`。
3. 将 `OS chain A status` 改为 `Normal`（如有 B，也改为 `Normal`）。
4. 将 `L4T Boot Mode` 设为 `ExtLinux`。
5. `Save & Exit` 后重启。

如果保存时报 `Submit Fail For Form: Grace Configuration`，说明 QSPI 里的
启动变量已损坏，只能通过 SDK Manager 完整重刷（最后手段）。

## 验证

执行修复脚本后重启，插着 MID360 观察：

- 开机不再长时间停在画面；
- 进入系统后 `efibootmgr` 的 `BootOrder` 中不再包含
  `0004/0003/0002/0005`；
- `cat /proc/cmdline` 仍为 `FS2:\boot\Image ...`，说明走的是
  `startup.nsh` 兼容路径；
- 若完成了 UEFI 菜单修复，`BootCurrent` 应变为 `0009`，并且
  `systemctl status nv-l4t-bootloader-config` 不再失败。

## 相关文件

| 路径 | 说明 |
|---|---|
| `tools/fix_mid360_boot.sh` | BootOrder 修复脚本（可 dry-run / 回滚） |
| `/startup.nsh` | UEFI Shell 自动执行的内核引导脚本 |
| `/boot/extlinux/extlinux.conf` | L4TLauncher 读取的启动配置 |
| `/home/argen/boot-issue-handoff.md` | 之前的完整排查交接文档 |
| `/home/argen/boot-fix-backup/` | 原始 EFI 变量、ESP 引导器与 RESCUE 记录 |
