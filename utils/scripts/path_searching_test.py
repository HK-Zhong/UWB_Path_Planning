import os
import numpy as np
import matplotlib.pyplot as plt

from grid_map import GridMap
from path_searching import EDTAwareAStarPlanner, GridAStarPlanner


"""
功能：
1. 读取已保存的 grid_map / edt_map
2. 在同一张地图上分别运行传统 A* 与 EDT-aware A*
3. 生成论文可用的三联图：
   (a) Binary Map
   (b) A* Path
   (c) EDT-aware Path
4. 图片保存到同级目录 ./pics/figures/
"""


def load_grid_map(grid_map_path: str,
                  edt_map_path: str,
                  resolution: float = 0.5) -> GridMap:
    """读取 numpy 地图并构造 GridMap 对象。"""
    grid_map_np = np.load(grid_map_path)
    edt_map_np = np.load(edt_map_path)

    print("[INFO] grid_map shape:", grid_map_np.shape)
    print("[INFO] edt_map  shape:", edt_map_np.shape)
    print("[INFO] edt max / min:", float(edt_map_np.max()), float(edt_map_np.min()))

    size = grid_map_np.shape[0] * resolution
    gm = GridMap(size=size, resolution=resolution)
    gm.grid_map = grid_map_np.copy()
    gm.edt_map = edt_map_np.copy()
    return gm



def plan_paths(gm: GridMap,
               start_world: tuple,
               goal_world: tuple,
               edt_hard_min: float = 0.5,
               w_safe: float = 5.0):
    """分别运行传统 A* 与 EDT-aware A*。"""
    start = gm.to_grid(*start_world)
    goal = gm.to_grid(*goal_world)

    print("[INFO] start:", start, "goal:", goal)

    # 传统 A*
    vanilla_astar = GridAStarPlanner(grid_map=gm, allow_diagonal=True)
    path_astar = vanilla_astar.plan(start, goal)

    # EDT-aware A*
    edt_astar = EDTAwareAStarPlanner(grid_map=gm, edt_hard_min=edt_hard_min)
    edt_astar.w_dist = 1.0
    edt_astar.w_safe = w_safe
    path_edt = edt_astar.plan(start, goal)

    if path_astar is None:
        print("[WARN] Vanilla A* failed to find a path.")
    else:
        print("[INFO] Vanilla A* path length:", len(path_astar))

    if path_edt is None:
        print("[WARN] EDT-aware A* failed to find a path.")
    else:
        print("[INFO] EDT-aware A* path length:", len(path_edt))

    return start, goal, path_astar, path_edt



def grid_path_to_xy(path):
    if path is None or len(path) == 0:
        return None, None
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    return xs, ys



def setup_axis(ax, title: str, sublabel: str):
    ax.set_title(title, fontsize=12)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.5, -0.10, sublabel,
            transform=ax.transAxes,
            ha="center", va="top",
            fontsize=11)



def draw_binary_map(ax, gm: GridMap):
    # grid_map == 1 为不可通行/障碍，显示为黑色
    # grid_map == 0 为自由/已知区域，显示为白色
    binary_map_vis = np.where(gm.grid_map.T == 1, 0.0, 1.0)
    ax.imshow(binary_map_vis, origin="lower", cmap="gray", vmin=0.0, vmax=1.0)



def draw_start_goal(ax, start, goal):
    ax.scatter(start[0], start[1], c="red", s=55, marker="o", zorder=5)
    ax.scatter(goal[0], goal[1], c="red", s=65, marker="x", zorder=5)
    ax.text(start[0] - 1.5, start[1] - 1.5, "S", color="red", fontsize=11, fontweight="bold")
    ax.text(goal[0] + 0.6, goal[1] + 0.6, "G", color="red", fontsize=11, fontweight="bold")



def plot_triptych(gm: GridMap,
                  start,
                  goal,
                  path_astar,
                  path_edt,
                  save_dir: str):
    """绘制三联图：(a) Binary Map, (b) A* Path, (c) EDT-aware Path。"""
    os.makedirs(save_dir, exist_ok=True)

    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))

    # ---------------- (a) Binary Map ----------------
    ax = axes[0]
    draw_binary_map(ax, gm)
    draw_start_goal(ax, start, goal)
    setup_axis(ax, "二值可通行地图", "(a) Binary Map")

    # ---------------- (b) A* Path ----------------
    ax = axes[1]
    draw_binary_map(ax, gm)
    draw_start_goal(ax, start, goal)
    xs, ys = grid_path_to_xy(path_astar)
    if xs is not None:
        ax.plot(xs, ys, color="#1f77b4", linewidth=2.5, label="A* 路径", zorder=4)
        ax.plot(xs, ys, "o", color="#1f77b4", markersize=2.8, zorder=4)
        ax.legend(loc="upper left", fontsize=9, frameon=True)
    setup_axis(ax, "传统 A* 路径", "(b) A* Path")

    # ---------------- (c) EDT-aware Path ----------------
    ax = axes[2]
    edt_im = ax.imshow(gm.edt_map.T, origin="lower", cmap="jet")
    obstacle_mask = np.where(gm.grid_map.T == 1, 1.0, np.nan)
    ax.imshow(obstacle_mask, origin="lower", cmap="gray", alpha=0.75)
    draw_start_goal(ax, start, goal)
    xs, ys = grid_path_to_xy(path_edt)
    if xs is not None:
        ax.plot(xs, ys, color="#ff7f0e", linewidth=2.7, label="EDT-A* 路径", zorder=4)
        ax.plot(xs, ys, "o", color="#ff7f0e", markersize=2.8, zorder=4)
        ax.legend(loc="upper left", fontsize=9, frameon=True)
    setup_axis(ax, "基于 EDT 的路径", "(c) EDT-aware Path")

    cbar = fig.colorbar(edt_im, ax=axes[2], fraction=0.046, pad=0.04)
    cbar.set_label("EDT (m)", fontsize=10)

    plt.tight_layout()

    svg_path = os.path.join(save_dir, "EDT路径偏移效果对比图.svg")
    pdf_path = os.path.join(save_dir, "EDT路径偏移效果对比图.pdf")
    png_path = os.path.join(save_dir, "EDT路径偏移效果对比图.png")

    plt.savefig(svg_path, bbox_inches="tight")
    plt.savefig(pdf_path, bbox_inches="tight")
    plt.savefig(png_path, dpi=300, bbox_inches="tight")
    plt.show()

    print("[INFO] Saved figure:", svg_path)
    print("[INFO] Saved figure:", pdf_path)
    print("[INFO] Saved figure:", png_path)



def main():
    # 1. 读取地图
    grid_map_path = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/scripts/pics/grid_map/grid_20260312_204230.npy"
    edt_map_path = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/scripts/pics/edt_map/edt_20260312_204230.npy"
    save_dir = "pics/figures"

    gm = load_grid_map(
        grid_map_path=grid_map_path,
        edt_map_path=edt_map_path,
        resolution=0.5,
    )

    # 2. 起点 / 终点（世界坐标）
    start_world = (-15, -15)
    goal_world = (18, 5)

    start, goal, path_astar, path_edt = plan_paths(
        gm=gm,
        start_world=start_world,
        goal_world=goal_world,
        edt_hard_min=0.5,
        w_safe=5.0,
    )

    # 3. 画三联图
    plot_triptych(
        gm=gm,
        start=start,
        goal=goal,
        path_astar=path_astar,
        path_edt=path_edt,
        save_dir=save_dir,
    )


if __name__ == "__main__":
    main()
