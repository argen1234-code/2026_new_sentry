#!/usr/bin/env python3
"""静止标定：从一次「车不动」的 rosbag 里测量地面与遮挡区。

用法:
    python3 calib_ground.py <bag_dir>

测量内容
  1. 遮挡区 / 近场自返回点（支架环）的半径与高度分布
  2. 雷达参考系到地面的距离（三种互不依赖的拟合方法 + 一致性判定）
  3. 地面在 odom 系的倾角（重力对齐是否成立）
  4. 地面在 base_footprint 里的高度（决定 pointcloud_to_laserscan 的高度闸门）
  5. /terrain_map_ext 的地面误报，按距离分桶（查"幻影圈"）

依赖的 bag 话题
    /livox/lidar/pointcloud  原始点云（未经 blind 过滤）—— 主标定
    /registered_scan         用于 odom 系交叉验证
    /lidar_odometry          用于取 vehicleZ
    /tf                      用于取 odom->base_footprint
    /terrain_map_ext         用于地面误报统计（无则跳过）

采集时请保持静止、位于"中间平整地面、两侧有障碍"的场景。
"""
import sys

import numpy as np
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


# ---------------------------------------------------------------- 读取 ----

def parse_livox_xyz(msg):
    """Livox 原始 PointCloud2 字段类型混杂（float32/uint8/float64），
    sensor_msgs_py.read_points_numpy 会抛 AssertionError，必须手工解析。"""
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step)
    rec = buf.reshape(n, msg.point_step)
    return rec[:, 0:12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)


def parse_xyz(msg):
    a = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
    return a.astype(np.float64).reshape(-1, 3)


def parse_xyzi(msg):
    a = pc2.read_points_numpy(msg, field_names=("x", "y", "z", "intensity"))
    return a.astype(np.float64).reshape(-1, 4)


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    raw, reg, ext = [], [], []
    odom_z, base_z = [], []
    kr = kg = ke = 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/livox/lidar/pointcloud":
            kr += 1
            if kr % 10:                     # 1/10 帧足够，且省内存
                continue
            a = parse_livox_xyz(deserialize_message(data, PointCloud2))
            if a.size:
                raw.append(a[~np.all(a == 0, axis=1)])   # 约 17% 是 (0,0,0)
        elif topic == "/registered_scan":
            kg += 1
            if kg % 5 == 0:
                a = parse_xyz(deserialize_message(data, PointCloud2))
                if a.size:
                    reg.append(a[~np.all(a == 0, axis=1)])
        elif topic == "/terrain_map_ext":
            ke += 1
            if ke % 5 == 0:
                a = parse_xyzi(deserialize_message(data, PointCloud2))
                if a.size:
                    ext.append(a)
        elif topic == "/lidar_odometry":
            m = deserialize_message(data, Odometry)
            odom_z.append(m.pose.pose.position.z)
        elif topic == "/tf":
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
                    base_z.append(tr.transform.translation.z)
    out = {
        "raw": np.concatenate(raw) if raw else np.zeros((0, 3)),
        "reg": np.concatenate(reg) if reg else np.zeros((0, 3)),
        "ext": np.concatenate(ext) if ext else np.zeros((0, 4)),
        "odom_z": np.asarray(odom_z),
        "base_z": np.asarray(base_z),
    }
    return out


# ---------------------------------------------------------------- 拟合 ----

def fit_ransac(P, tol=0.02, zmin=0.95, iters=600, seed=1):
    """完全自由的平面拟合。注意：在很大的 z 窗口上跑会得到"桥接平面"
    （近处贴地、远处爬上障碍物），见文档 §3.2 的判伪方法。"""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        p = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if abs(n[2]) < zmin:
            continue
        if n[2] < 0:
            n = -n
        c = -n @ p[0]
        inl = int((np.abs(P @ n + c) < tol).sum())
        if best is None or inl > best[0]:
            best = (inl, n, c)
    if best is None:
        return None
    inl, n, c = best
    for _ in range(5):
        m = np.abs(P @ n + c) < tol
        if m.sum() < 10:
            break
        cen = P[m].mean(0)
        cov = (P[m] - cen).T @ (P[m] - cen) / m.sum()
        _, vt = np.linalg.eigh(cov)
        n = vt[:, 0]
        n = n if n[2] > 0 else -n
        c = -n @ cen
    m = np.abs(P @ n + c) < tol
    rms = float(np.sqrt(((P[m] @ n + c) ** 2).mean())) if m.sum() else np.nan
    return {"d": float(-c), "n": n, "c": c,
            "tilt_deg": float(np.degrees(np.arccos(abs(n[2])))),
            "inliers": int(m.sum()), "total": int(len(P)), "rms": rms}


def fit_flatcells(P, cell=0.05, thick=0.03, minpts=5, tol=0.02, iters=2000, seed=21):
    """主方法：只保留"纯地面格"再拟合。

    纯地面格 = 5 cm 网格内点数 >= minpts，且该格的 z 极差 < thick。
    障碍物的体素里 z 极差一定大（有底边也有侧面/顶面），会被自然剔除；
    地板格则几乎是平的。这样得到的点集只剩地面本体，不会被障碍物底边
    或远处障碍物顶部"桥接"。

    返回: z0(原点处平面高度)、法向、倾角、pitch/roll 分解、按距离的残差表。
    """
    if len(P) < 100:
        return None
    key = (np.floor(P[:, 0] / cell).astype(np.int64) * 100000
           + np.floor(P[:, 1] / cell).astype(np.int64))
    uk, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    order = np.lexsort((P[:, 2], inv))
    zs, iv = P[order, 2], inv[order]
    st = np.searchsorted(iv, np.arange(len(uk)))
    zmin = zs[st]
    zmax = np.array([zs[s + c - 1] for s, c in zip(st, cnt)])
    mean_z = np.array([zs[s:s + c].mean() for s, c in zip(st, cnt)])
    keep = (cnt >= minpts) & ((zmax - zmin) < thick)
    if keep.sum() < 50:
        return None
    E = np.stack([uk[keep] // 100000 * cell + cell / 2,
                  uk[keep] % 100000 * cell + cell / 2,
                  mean_z[keep]], axis=1)
    fit = fit_ransac(E, tol=tol, zmin=0.90, iters=iters, seed=seed)
    if fit is None:
        return None
    n = fit["n"]
    m = np.abs(E @ n + fit["c"]) < tol
    res = E[m] @ n + fit["c"]
    rr = np.hypot(E[m, 0], E[m, 1])
    fit["z0"] = float(-fit["c"] / n[2])          # 原点处的地面高度
    fit["pitch_deg"] = float(np.degrees(np.arctan2(n[0], n[2])))
    fit["roll_deg"] = float(np.degrees(np.arctan2(-n[1], n[2])))
    fit["cells"] = int(keep.sum())
    fit["drift"] = []
    for lo, hi in ((1.2, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 3.5), (3.5, 5.0)):
        s = (rr > lo) & (rr < hi)
        if s.sum() >= 20:
            fit["drift"].append((lo, hi, float(np.median(res[s])), int(s.sum())))
    return fit


# ---------------------------------------------------------------- 报告 ----

def fmt(fit):
    return ("d=%.3f m  tilt=%.2f°  inliers=%d/%d  rms=%.1f mm"
            % (fit["d"], fit["tilt_deg"], fit["inliers"], fit["total"],
               fit["rms"] * 1000))


def main(path):
    B = read_bag(path)
    raw, ext = B["raw"], B["ext"]
    print("=" * 72)
    print("bag: %s" % path)
    print("原始点云 %d 点 | registered_scan %d 点 | terrain_map_ext %d 点"
          % (len(raw), len(B["reg"]), len(ext)))

    if len(raw) == 0:
        print("!! 没有 /livox/lidar/pointcloud，无法标定地面")
        return 1

    # --- 1. 遮挡区 / 近场自返回 ---
    print("\n[1] 遮挡区 / 近场自返回")
    rr = np.hypot(raw[:, 0], raw[:, 1])
    near = (rr < 0.5) & (raw[:, 2] > -0.3) & (raw[:, 2] < 0.3)
    if near.sum():
        rn = rr[near]
        print("  r<0.5 m 的点 %d 个（%.1f%%），半径 p5/p50/p95 = %.3f/%.3f/%.3f m"
              % (near.sum(), 100 * near.mean(),
                 *np.percentile(rn, [5, 50, 95])))
        print("  z    p5/p50/p95 = %+.3f/%+.3f/%+.3f m"
              % tuple(np.percentile(raw[near, 2], [5, 50, 95])))
        print("  -> 这是支架/外壳自返回；Point-LIO 的 blind 必须大于它的最大半径，"
              "否则自返回点会进图")
    else:
        print("  r<0.5 m 内没有点（blind 或场景遮挡可能导致，需另行确认）")

    # --- 2. 地面拟合：主方法 = 纯地面格 ---
    print("\n[2] 雷达参考系到地面的距离")
    rr_all = np.hypot(raw[:, 0], raw[:, 1])
    band = (rr_all > 1.2) & (rr_all < 5.5) & (raw[:, 2] > -0.6) & (raw[:, 2] < 0.4)
    flat = fit_flatcells(raw[band])
    if flat:
        print("  [主方法 纯地面格]   原点处 z0=%.4f m  倾角=%.2f° (pitch %+.2f°, roll %+.2f°)"
              % (flat["z0"], flat["tilt_deg"], flat["pitch_deg"], flat["roll_deg"]))
        print("                      内点 %d/%d 格  rms=%.1f mm"
              % (flat["inliers"], flat["cells"], flat["rms"] * 1000))
        print("      按距离残差（应当接近 0 且不随距离漂移）:")
        for lo, hi, med, cnt in flat["drift"]:
            print("        r %.1f-%.1f m: %+.1f cm (n=%d)" % (lo, hi, med * 100, cnt))
    else:
        print("  [主方法] 纯地面格不足，场景里可能没有足够平整地面")

    # 交叉验证：宽窗口自由拟合（容易被障碍物桥接，只作对照）
    wide = fit_ransac(raw[band])
    if wide:
        n = wide["n"]
        rise = np.hypot(n[0], n[1]) / max(abs(n[2]), 1e-9) * (5.0 - 1.5)
        print("  [对照 宽窗自由拟合] z0=%.4f m  tilt=%.2f°  跨1.5-5.0m抬升 %.1f cm %s"
              % (-wide["c"] / n[2], wide["tilt_deg"], rise * 100,
                 "(>10 cm: 已与环境耦合，勿单独采信)" if rise > 0.10 else ""))

    # --- 3. odom 系交叉验证 ---
    print("\n[3] odom 系交叉验证")
    of = None
    ground_in_base = None
    if len(B["reg"]):
        reg = B["reg"]
        rr_r = np.hypot(reg[:, 0], reg[:, 1])
        m = (rr_r > 1.2) & (rr_r < 5.0) & (reg[:, 2] > -0.4) & (reg[:, 2] < 0.05)
        of = fit_ransac(reg[m])
        if of:
            print("  registered_scan 地面: %s" % fmt(of))
            gz = -of["c"] if of["n"][2] > 0 else of["c"]
            if len(B["base_z"]):
                ground_in_base = gz - float(np.median(B["base_z"]))
                print("  odom->base_footprint z 中位: %.4f m" % np.median(B["base_z"]))
                print("  地面在 base_footprint 的高度: %+.4f m  (判据 |x|<0.02)"
                      % ground_in_base)
            if len(B["odom_z"]):
                print("  lidar_odometry z 中位: %.4f m" % np.median(B["odom_z"]))
                print("  地面相对 vehicleZ: %+.4f m  (minRelZ/lowerBoundZ 的依据)"
                      % (gz - np.median(B["odom_z"])))
            # 反解真实雷达高度
            if ground_in_base is not None:
                # 地面在 base 的高度 = t_il - h + lidar_z  =>  h = t_il - lidar_z + g
                print("  提示：结合 URDF lidar_z 可反解真实高度，公式见文档 §4")

    # --- 4. 地面误报 / 幻影圈 ---
    if len(ext):
        print("\n[4] /terrain_map_ext 地面误报")
        X, I = ext[:, :3], ext[:, 3]
        rr_e = np.hypot(X[:, 0], X[:, 1])
        if of:
            hrel = X @ of["n"] + of["c"]
            grd = (np.abs(hrel) < 0.03) & (rr_e > 1.5)
            print("  地面点占输出 %.1f%%；经 intensity>=0.10 闸门后误报 %.2f%%"
                  % (100 * grd.mean(), 100 * np.mean((I >= 0.10) & grd)))
            mark = grd & (I >= 0.10)
            if mark.sum():
                hist, edges = np.histogram(rr_e[mark],
                                           bins=[1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 8])
                print("  误标点的距离分布（幻影圈定位）:")
                for a, b, v in zip(edges[:-1], edges[1:], hist):
                    if v:
                        print("      %.1f-%.1f m: %d" % (a, b, v))
                cell = (np.floor(X[:, 0] / 0.05).astype(np.int64) * 100000
                        + np.floor(X[:, 1] / 0.05).astype(np.int64))
                print("  误标 5cm 栅格数: %d" % len(np.unique(cell[mark])))
    print("=" * 72)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    sys.exit(main(sys.argv[1]))
