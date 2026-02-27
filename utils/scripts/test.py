import numpy as np
import matplotlib.pyplot as plt

from grid_map import GridMap
from path_searching import AStar


def main():
    # ===============================
    # 1. 读取保存的 numpy 数据
    # ===============================
    grid_map_path = "pics/grid_map/grid_20260226_212734.npy"
    edt_map_path  = "pics/edt_map/edt_20260226_212734.npy"

    grid_map_np = np.load(grid_map_path)
    edt_map_np  = np.load(edt_map_path)

    print("[TEST] grid_map shape:", grid_map_np.shape)
    print("[TEST] edt_map  shape:", edt_map_np.shape)
    print("[TEST] edt max / min:", edt_map_np.max(), edt_map_np.min())

    # ===============================
    # 2. 构造 GridMap
    # ===============================
    resolution = 0.5
    size = grid_map_np.shape[0] * resolution

    gm = GridMap(size=size, resolution=resolution)
    gm.grid_map = grid_map_np.copy()
    gm.edt_map  = edt_map_np.copy()

    # ===============================
    # 3. AStar
    # ===============================
    astar = AStar(
        grid_map=gm,
        edt_hard_min=resolution * 1.5
    )

    astar.w_dist = 1.0
    astar.w_safe = 5.0   # 可以改成 10 / 20 对比

    # ===============================
    # 4. 起点 & 终点（真实 → 栅格）
    # ===============================
    start = gm.to_grid(-15, -15)
    goal  = gm.to_grid(18, 5)

    print("[TEST] start:", start, "goal:", goal)

    # ===============================
    # 5. 搜索路径
    # ===============================
    path = astar.plan(start, goal)

    if path is None:
        print("[TEST] ❌ No path found.")
        return

    print("[TEST] ✅ Path found, length =", len(path))

    # =====================================================
    # 🔴 6. 找出 EDT <= 0.5 的“危险路径点”
    # =====================================================
    danger_pts = []
    for (gx, gy) in path:
        edt = gm.edt_map[gx, gy]
        if edt < 1.0:
            danger_pts.append((gx, gy, edt))

    print(f"[TEST] ⚠️ EDT <= 0.5 points: {len(danger_pts)}")
    for gx, gy, edt in danger_pts:
        print(f"  grid=({gx},{gy})  edt={edt:.3f} m")

    # ===============================
    # 7. 可视化
    # ===============================
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]

    plt.figure(figsize=(7, 7))

    # EDT 背景
    plt.imshow(
        gm.edt_map.T,
        origin="lower",
        cmap="jet"
    )
    plt.colorbar(label="EDT (m)")

    # A* 路径
    plt.plot(xs, ys, "-r", linewidth=2, label="A* path")

    # 起点终点
    plt.scatter(start[0], start[1],
                c="lime", s=80, marker="o", label="start")
    plt.scatter(goal[0], goal[1],
                c="cyan", s=80, marker="x", label="goal")

    # =====================================================
    # 🔴 危险点高亮（EDT <= 0.5）
    # =====================================================
    if danger_pts:
        dx = [p[0] for p in danger_pts]
        dy = [p[1] for p in danger_pts]
        plt.scatter(
            dx, dy,
            c="black",
            s=40,
            marker="s",
            label="EDT <= 0.5"
        )

    plt.legend()
    plt.title("A* path with EDT <= 0.5 points highlighted")
    plt.axis("equal")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
