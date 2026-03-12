import yaml
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import traceback
import rospy

from scipy.ndimage import distance_transform_edt


class GridMap:
    """
    GridMap:
    - self.grid_map : LOS-based binary map (0 free / 1 unknown or obstacle)
    - self.edt_map  : Euclidean Distance Transform (meters), derived from grid_map
    """

    def __init__(self, size=50, resolution=0.5, map_no=1):
        """
        size        : map size (meters)
        resolution  : grid resolution (meters)
        """
        self.size = size
        self.resolution = resolution
        self.grid_size = int(size / resolution)

        # -------------------------------
        # Binary grid map (SYSTEM INTERFACE)
        # 0 = free (LOS confirmed)
        # 1 = unknown / obstacle
        # -------------------------------
        self.grid_map = np.ones(
            (self.grid_size, self.grid_size), dtype=np.int8
        )

        # -------------------------------
        # EDT map (meters)
        # -------------------------------
        self.edt_map = None

        # ---------- UWB anchors ----------
        # Anchor LOS connectivity prior
        if map_no == 1:
            self.uwb_anchors_file = (
                "/home/coolas-fly/UWB_Path_Planning/src/"
                "UWB_Path_Planning/utils/config/UWB_Anchors.yml"
            )
            self.uwb_los = [
                (0, 1), (1, 2), (2, 3), (1, 4),
                (2, 5), (3, 6), (3, 8), (0, 7),
                (7, 9), (9, 10), (9, 11), (11, 12),
                (12, 13), (11, 13), (8, 13),
                (4, 5), (5, 6), (10, 12)
            ]
        else:
            self.uwb_anchors_file = (
                "/home/coolas-fly/UWB_Path_Planning/src/"
                "UWB_Path_Planning/utils/config/UWB_Anchors2.yml"
            )
            self.uwb_los = [
                (0, 4), (1, 2), (1, 5), (2, 6),
                (3, 4), (3, 8), (4, 5), (5, 6),
                (5, 10), (6, 7), (7, 11), (8, 9),
                (8, 15), (9, 10), (10, 12), (11, 13),
                (12, 13), (12, 16), (14, 15), (14, 18),
                (15, 16), (16, 17), (16, 19)
            ]

        self.real_anchors_position = self.load_uwb_anchors()

    # =====================================================
    # Coordinate transform
    # =====================================================
    def to_grid(self, x_real, y_real):
        gx = int((x_real + self.size / 2.0) / self.resolution)
        gy = int((y_real + self.size / 2.0) / self.resolution)
        return gx, gy

    def to_real(self, gx, gy):
        x = gx * self.resolution - self.size / 2.0
        y = gy * self.resolution - self.size / 2.0
        return round(x, 2), round(y, 2)

    # =====================================================
    # UWB anchor loading
    # =====================================================
    def load_uwb_anchors(self):
        anchors = {}
        with open(self.uwb_anchors_file, "r") as f:
            data = yaml.safe_load(f)
        for a in data["UWB_Anchors"]:
            anchors[a["id"]] = (a["x"], a["y"])
        return anchors

    # =====================================================
    # LOS map update
    # =====================================================
    def get_line_grids(self, start_real, end_real):
        """Bresenham line algorithm"""
        x0, y0 = self.to_grid(*start_real)
        x1, y1 = self.to_grid(*end_real)

        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def map_update(self, start_real, end_real, free_expand=1):
        """
        Update grid_map using LOS ray
        """
        for gx, gy in self.get_line_grids(start_real, end_real):
            for dx in range(-free_expand, free_expand + 1):
                for dy in range(-free_expand, free_expand + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        self.grid_map[nx, ny] = 0

    def map_update_by_los(self, uav_pos_real, los_data, free_expand=1):
        for tag in los_data:
            if tag["LOS"]:
                self.map_update(
                    uav_pos_real,
                    self.real_anchors_position[tag["id"]],
                    free_expand,
                )

    def map_init(self, free_expand=1):
        for i, j in self.uwb_los:
            self.map_update(
                self.real_anchors_position[i],
                self.real_anchors_position[j],
                free_expand,
            )

    # =====================================================
    # EDT maintenance (NO map erosion)
    # =====================================================
    def update_edt(self):
        """
        Compute EDT from grid_map
        edt_map unit: meters
        """
        obstacle_mask = (self.grid_map == 1)
        dist_cells = distance_transform_edt(~obstacle_mask)
        self.edt_map = dist_cells * self.resolution

    def is_safe(self, gx, gy, safe_distance_m):
        """
        Safety query using EDT
        """
        if self.edt_map is None:
            return False
        return self.edt_map[gx, gy] >= safe_distance_m

    # =====================================================
    # Visualization
    # =====================================================
    def get_pic_dir(self):
        d = os.path.join(os.path.dirname(__file__), "pics")
        os.makedirs(d, exist_ok=True)
        os.makedirs(os.path.join(d, "grid_map"), exist_ok=True)
        os.makedirs(os.path.join(d, "edt_map"), exist_ok=True)
        os.makedirs(os.path.join(d, "pipeline"), exist_ok=True)
        return d

    def _draw_path_points(self, path_points):
        """
        路径可视化规则：
        - 起点、终点：红色叉
        - 中间点：蓝色圆
        """
        # ===== 可调参数（路径点样式接口） =====
        mid_color = "red"      # 中间路径点颜色
        mid_size = 5           # 中间路径点大小
        start_end_size = 50    # 起点终点大小

        if not path_points:
            return

        if len(path_points) == 1:
            x, y = path_points[0]
            plt.scatter([x], [y], c="red", marker="x", s=60, linewidths=2)
            return

        start = path_points[0]
        end = path_points[-1]
        mid = path_points[1:-1]

        if mid:
            xs = [p[0] for p in mid]
            ys = [p[1] for p in mid]
            plt.scatter(xs, ys, c=mid_color, marker="o", s=mid_size)

        plt.scatter([start[0]], [start[1]], c="red", marker="x", s=start_end_size, linewidths=2)
        plt.scatter([end[0]], [end[1]], c="red", marker="x", s=start_end_size, linewidths=2)

    def _save_pipeline_single_map(self, data, path_points, pic_path, npy_path, csv_path, title, cmap, add_colorbar=False):
        np.save(npy_path, data)

        if path_points:
            np.savetxt(
                csv_path,
                np.array(path_points, dtype=np.int32),
                fmt="%d",
                delimiter=",",
                header="gx,gy",
                comments=""
            )

        plt.figure(figsize=(6, 6))
        plt.imshow(data.T, cmap=cmap, origin="lower")

        if add_colorbar:
            plt.colorbar(label="distance to obstacle (m)")

        self._draw_path_points(path_points)

        # plt.title(title)
        plt.tight_layout()
        plt.savefig(pic_path, dpi=200)
        plt.close()

    def save_pipeline_visualization(self, path_points, tag):
        """
        保存 pipeline 可视化结果到 pics/pipeline/ 目录：
        - 当前 grid map 图像 + 矩阵
        - 当前 edt map 图像 + 矩阵
        - 当前离散路径 csv
        """
        if self.grid_map is None or self.edt_map is None:
            rospy.logwarn("[GridMap] grid_map or edt_map is empty, skip pipeline visualization.")
            return

        pipeline_dir = os.path.join(self.get_pic_dir(), "pipeline")
        os.makedirs(pipeline_dir, exist_ok=True)

        grid_pic = os.path.join(pipeline_dir, f"pipeline_grid_{tag}.png")
        edt_pic = os.path.join(pipeline_dir, f"pipeline_edt_{tag}.png")
        grid_npy = os.path.join(pipeline_dir, f"pipeline_grid_{tag}.npy")
        edt_npy = os.path.join(pipeline_dir, f"pipeline_edt_{tag}.npy")
        grid_csv = os.path.join(pipeline_dir, f"pipeline_grid_path_{tag}.csv")
        edt_csv = os.path.join(pipeline_dir, f"pipeline_edt_path_{tag}.csv")

        self._save_pipeline_single_map(
            data=self.grid_map,
            path_points=path_points,
            pic_path=grid_pic,
            npy_path=grid_npy,
            csv_path=grid_csv,
            title=f"Grid Map with Planned Path ({tag})",
            cmap="gray_r",
            add_colorbar=False,
        )

        self._save_pipeline_single_map(
            data=self.edt_map,
            path_points=path_points,
            pic_path=edt_pic,
            npy_path=edt_npy,
            csv_path=edt_csv,
            title=f"EDT Map with Planned Path ({tag})",
            cmap="jet",
            add_colorbar=True,
        )

        rospy.loginfo(f"[GridMap] Pipeline visualization saved:")
        rospy.loginfo(f"  - {grid_pic}")
        rospy.loginfo(f"  - {edt_pic}")

    def visualize(self, highlight_points=None, filename="grid"):

        ts = time.strftime("%Y%m%d_%H%M%S")
        pic_path = os.path.join(self.get_pic_dir(), "grid_map", f"{filename}_{ts}.png")

        # ===== 保存矩阵路径 =====
        npy_path = os.path.join(self.get_pic_dir(), "grid_map", f"{filename}_{ts}.npy")
        # txt_path = os.path.join(self.get_pic_dir(), f"{filename}_{ts}.txt")

        # ===== 保存 grid_map 矩阵 =====
        np.save(npy_path, self.grid_map)

        # ===== 保存 highlight_points（如果有） =====
        if highlight_points:
            hp_path = os.path.join(self.get_pic_dir(), "grid_map", f"{filename}_{ts}_highlight_points.npy")
            np.save(hp_path, np.array(highlight_points, dtype=np.int32))
            # 同时保存为可直接查看的 CSV
            hp_csv_path = os.path.join(self.get_pic_dir(), "grid_map", f"{filename}_{ts}_highlight_points.csv")
            np.savetxt(hp_csv_path, np.array(highlight_points, dtype=np.int32), fmt="%d", delimiter=",", header="gx,gy",
                       comments="")
            print(f"[GridMap] highlight_points saved:")
            print(f"  - {hp_path}")
            print(f"  - {hp_csv_path}")

        print(f"[GridMap] grid_map matrix saved:")
        print(f"  - {npy_path}")
        # print(f"  - {txt_path}")

        # ===== 可视化 =====
        plt.figure(figsize=(6, 6))
        plt.imshow(self.grid_map.T, cmap="gray_r", origin="lower")

        if highlight_points:
            xs = [p[0] for p in highlight_points]
            ys = [p[1] for p in highlight_points]
            plt.scatter(xs, ys, c="red", s=10)

        plt.xticks(np.arange(-0.5, self.grid_size, 1), [])
        plt.yticks(np.arange(-0.5, self.grid_size, 1), [])
        plt.grid(color='black', linestyle='-', linewidth=0.5)

        plt.savefig(pic_path, dpi=200)
        plt.close()

        print("[GridMap] grid_map image saved:", pic_path)

    def visualize_edt(self, highlight_points=None, filename="edt"):
        if self.edt_map is None:
            print("[GridMap] edt_map empty, call update_edt() first.")
            return

        ts = time.strftime("%Y%m%d_%H%M%S")
        pic_path = os.path.join(self.get_pic_dir(), "edt_map", f"{filename}_{ts}.png")

        # ===== 保存矩阵路径 =====
        npy_path = os.path.join(self.get_pic_dir(), "edt_map", f"{filename}_{ts}.npy")
        # txt_path = os.path.join(self.get_pic_dir(), f"{filename}_{ts}.txt")

        # ===== 保存 edt_map 矩阵 =====
        np.save(npy_path, self.edt_map)

        # ===== 保存 highlight_points（如果有） =====
        if highlight_points:
            hp_path = os.path.join(self.get_pic_dir(), "edt_map", f"{filename}_{ts}_highlight_points.npy")
            np.save(hp_path, np.array(highlight_points, dtype=np.int32))
            # 同时保存为可直接查看的 CSV
            hp_csv_path = os.path.join(self.get_pic_dir(), "edt_map", f"{filename}_{ts}_highlight_points.csv")
            np.savetxt(hp_csv_path, np.array(highlight_points, dtype=np.int32), fmt="%d", delimiter=",", header="gx,gy",
                       comments="")
            print(f"[GridMap] highlight_points saved:")
            print(f"  - {hp_path}")
            print(f"  - {hp_csv_path}")

        print(f"[GridMap] edt_map matrix saved:")
        print(f"  - {npy_path}")
        # print(f"  - {txt_path}")

        # ===== 可视化 =====
        plt.figure(figsize=(6, 6))
        plt.imshow(self.edt_map.T, cmap="jet", origin="lower")
        plt.colorbar(label="distance to obstacle (m)")

        if highlight_points:
            xs = [p[0] for p in highlight_points]
            ys = [p[1] for p in highlight_points]
            plt.scatter(xs, ys, c="red", s=10)

        plt.savefig(pic_path, dpi=200)
        plt.close()

        print("[GridMap] edt_map image saved:", pic_path)


# =====================================================
# Standalone test
# =====================================================
if __name__ == "__main__":
    gm = GridMap(size=50, resolution=0.5)

    print("[1] Init LOS map")
    gm.map_init(free_expand=1)

    print("[2] Update EDT")
    gm.update_edt()

    test_points = [(30, 40), (40, 45), (50, 55)]

    gm.visualize(highlight_points=test_points, filename="los_map")
    gm.visualize_edt(highlight_points=test_points, filename="edt_map")
