#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import yaml


def load_xyz_from_csv(file_path):
    """
    从 CSV 中读取 xyz 坐标。
    支持两种常见格式：
    1) time, x, y, z
    2) x, y, z
    自动跳过表头。
    """
    xs, ys, zs = [], [], []

    with open(file_path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue

            # 去掉空白字符
            row = [item.strip() for item in row]

            # 跳过表头
            try:
                values = [float(v) for v in row]
            except ValueError:
                continue

            if len(values) >= 4:
                # 默认认为是 time, x, y, z
                x, y, z = values[1], values[2], values[3]
            elif len(values) >= 3:
                # 默认认为是 x, y, z
                x, y, z = values[0], values[1], values[2]
            else:
                continue

            xs.append(x)
            ys.append(y)
            zs.append(z)

    return xs, ys, zs


def create_box_faces(cx, cy, cz, sx, sy, sz):
    x_min, x_max = cx - sx / 2.0, cx + sx / 2.0
    y_min, y_max = cy - sy / 2.0, cy + sy / 2.0
    z_min, z_max = cz - sz / 2.0, cz + sz / 2.0

    v000 = [x_min, y_min, z_min]
    v001 = [x_min, y_min, z_max]
    v010 = [x_min, y_max, z_min]
    v011 = [x_min, y_max, z_max]
    v100 = [x_max, y_min, z_min]
    v101 = [x_max, y_min, z_max]
    v110 = [x_max, y_max, z_min]
    v111 = [x_max, y_max, z_max]

    return [
        [v000, v100, v110, v010],
        [v001, v101, v111, v011],
        [v000, v100, v101, v001],
        [v010, v110, v111, v011],
        [v000, v010, v011, v001],
        [v100, v110, v111, v101],
    ]


def load_and_draw_obstacles(ax, yml_path):
    with open(yml_path, 'r') as f:
        data = yaml.safe_load(f)

    obstacles = data.get("obstacles", [])

    first = True
    for obs in obstacles:
        faces = create_box_faces(
            obs["cx"], obs["cy"], obs["cz"],
            obs["sx"], obs["sy"], obs["sz"]
        )

        poly = Poly3DCollection(
            faces,
            alpha=0.2,
            edgecolor='gray',
            linewidths=0.5,
            label='Obstacles' if first else None
        )
        poly.set_facecolor('lightgray')
        ax.add_collection3d(poly)
        first = False


def main():
    base_dir = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/scripts/waypoints"

    file_bs = os.path.join(base_dir, "waypoints_bs.csv")
    file_ego = os.path.join(base_dir, "waypoints_ego.csv")
    file_ours = os.path.join(base_dir, "waypoints_ours.csv")

    for file_path in [file_bs, file_ego, file_ours]:
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"文件不存在: {file_path}")

    bs_x, bs_y, bs_z = load_xyz_from_csv(file_bs)
    ego_x, ego_y, ego_z = load_xyz_from_csv(file_ego)
    ours_x, ours_y, ours_z = load_xyz_from_csv(file_ours)

    # 创建保存目录
    save_dir = os.path.join(os.getcwd(), "pics", "traj_vis")
    os.makedirs(save_dir, exist_ok=True)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 三条轨迹
    ax.plot(bs_x, bs_y, bs_z, label="Baseline", linewidth=2)
    ax.plot(ego_x, ego_y, ego_z, label="EGO-Planner", linewidth=2)
    ax.plot(ours_x, ours_y, ours_z, label="Ours", linewidth=2)

    # 读取并绘制障碍物
    obstacle_file = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/indoor_environment2_obstacle.yml"
    load_and_draw_obstacles(ax, obstacle_file)

    # 起点和终点标记（可选但很有用）
    if bs_x:
        ax.scatter(bs_x[0], bs_y[0], bs_z[0], marker="o", s=40, label="Start")
        ax.scatter(bs_x[-1], bs_y[-1], bs_z[-1], marker="^", s=50, label="End")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    # ax.set_title("3D Trajectory Comparison")
    ax.legend()
    ax.grid(True)

    # 让 xyz 比例更自然一些
    try:
        all_x = bs_x + ego_x + ours_x
        all_y = bs_y + ego_y + ours_y
        all_z = bs_z + ego_z + ours_z

        # 额外加入障碍物中心用于缩放
        with open("/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/indoor_environment2_obstacle.yml", 'r') as f:
            obs_data = yaml.safe_load(f)
        for obs in obs_data.get("obstacles", []):
            all_x.append(obs["cx"])
            all_y.append(obs["cy"])
            all_z.append(obs["cz"])

        if all_x and all_y and all_z:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            z_min, z_max = min(all_z), max(all_z)

            max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
            x_mid = 0.5 * (x_max + x_min)
            y_mid = 0.5 * (y_max + y_min)
            z_mid = 0.5 * (z_max + z_min)

            ax.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
            ax.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
            ax.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)
    except Exception:
        pass

    plt.tight_layout()

    # 保存图片
    save_path = os.path.join(save_dir, "trajectory_comparison.png")
    plt.savefig(save_path, dpi=300)
    print(f"[INFO] Trajectory figure saved to: {save_path}")

    plt.show()


if __name__ == "__main__":
    main()