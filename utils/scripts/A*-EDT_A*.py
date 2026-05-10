import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.ndimage import distance_transform_edt

from grid_map import GridMap
from path_searching import EDTAwareAStarPlanner, GridAStarPlanner

# ===================== 基本设置 =====================
plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'SimHei', 'PingFang SC']
plt.rcParams['axes.unicode_minus'] = False

save_dir = './pics/figures'
os.makedirs(save_dir, exist_ok=True)

# ===================== 地图参数 =====================
W, H = 20, 14   # 地图宽高（栅格坐标系）
start = (2, 2)
goal = (17, 11)

# 障碍物定义：(x, y, w, h)
# 这里故意设计成“传统A*会贴边，而EDT路径会偏中间”的场景
obstacles = [
    (5, 3, 2, 8),    # 左中竖障碍
    (10, 6, 2, 5),   # 中部竖障碍
    (13, 2, 3, 3),   # 右下障碍
    (2, 10, 4, 2),   # 左上障碍
]

# ===================== 用真实算法生成路径 =====================
resolution = 1.0
size = max(W, H)

gm = GridMap(size=size, resolution=resolution)

# 先全部置为自由空间，再按当前障碍物定义写入障碍
if gm.grid_map.shape[0] < W or gm.grid_map.shape[1] < H:
    raise RuntimeError('GridMap 尺寸不足，请检查 size / resolution 设置。')

gm.grid_map[:, :] = 0
for ox, oy, ow, oh in obstacles:
    x0 = int(round(ox))
    y0 = int(round(oy))
    x1 = int(round(ox + ow))
    y1 = int(round(oy + oh))
    gm.grid_map[x0:x1, y0:y1] = 1

# 根据当前障碍物地图构建 EDT
free_mask = (gm.grid_map == 0)
gm.edt_map = distance_transform_edt(free_mask) * resolution

# 调用两种真实算法
vanilla_astar = GridAStarPlanner(grid_map=gm, allow_diagonal=True)
path_astar = vanilla_astar.plan(start, goal)

edt_astar = EDTAwareAStarPlanner(grid_map=gm, edt_hard_min=1.0)
edt_astar.w_dist = 1.0
edt_astar.w_safe = 5.0
path_edt = edt_astar.plan(start, goal)

if path_astar is None:
    raise RuntimeError('传统 A* 未找到可行路径。')
if path_edt is None:
    raise RuntimeError('EDT-aware A* 未找到可行路径。')

path_astar = np.array(path_astar)
path_edt = np.array(path_edt)

# ===================== 画图工具函数 =====================
def draw_obstacles(ax):
    for ox, oy, ow, oh in obstacles:
        rect = Rectangle((ox, oy), ow, oh, facecolor='black', edgecolor='black', alpha=0.9)
        ax.add_patch(rect)

def draw_start_goal(ax):
    ax.plot(start[0], start[1], 'o', color='red', markersize=7)
    ax.plot(goal[0], goal[1], 'o', color='red', markersize=7)
    ax.text(start[0]-0.6, start[1]-0.8, 'S', color='red', fontsize=11, fontweight='bold')
    ax.text(goal[0]+0.2, goal[1]+0.2, 'G', color='red', fontsize=11, fontweight='bold')

def setup_axis(ax, title, sublabel):
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=13)
    ax.text(0.5, -0.12, sublabel, transform=ax.transAxes,
            ha='center', va='top', fontsize=12)

# ===================== 正式画图 =====================
fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))

# -------- (a) 二值地图 --------
ax = axes[0]
draw_obstacles(ax)
draw_start_goal(ax)
setup_axis(ax, '二值可通行地图', '(a) Binary Map')

# -------- (b) 传统A*路径 --------
ax = axes[1]
draw_obstacles(ax)
draw_start_goal(ax)
ax.plot(path_astar[:, 0], path_astar[:, 1], '-', color='#1f77b4', linewidth=2.5, label='A* 路径')
ax.plot(path_astar[:, 0], path_astar[:, 1], 'o', color='#1f77b4', markersize=3)
setup_axis(ax, '传统 A* 路径', '(b) A* Path')
ax.legend(loc='upper left', fontsize=10, frameon=True)

# -------- (c) EDT-A*路径 --------
ax = axes[2]
# EDT 背景热力图（由当前障碍物地图真实计算得到）
im = ax.imshow(
    gm.edt_map.T,
    origin='lower',
    cmap='jet',
    alpha=0.78,
    aspect='auto'
)
draw_obstacles(ax)
draw_start_goal(ax)
ax.plot(path_edt[:, 0], path_edt[:, 1], '-', color='#ff7f0e', linewidth=2.8, label='EDT-A* 路径')
ax.plot(path_edt[:, 0], path_edt[:, 1], 'o', color='#ff7f0e', markersize=3)
setup_axis(ax, '基于 EDT 的路径', '(c) EDT-aware Path')
ax.legend(loc='upper left', fontsize=10, frameon=True)

# colorbar 只给第三幅图服务
cbar = fig.colorbar(im, ax=axes[2], fraction=0.046, pad=0.04)
cbar.set_label('EDT 距离值', fontsize=10)

plt.tight_layout()

# ===================== 保存 =====================
svg_path = os.path.join(save_dir, 'EDT路径偏移效果对比图.svg')
pdf_path = os.path.join(save_dir, 'EDT路径偏移效果对比图.pdf')

plt.savefig(svg_path, bbox_inches='tight')
plt.savefig(pdf_path, bbox_inches='tight')
plt.close()

print(f'图像已保存到: {svg_path}')
print(f'图像已保存到: {pdf_path}')