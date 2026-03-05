import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import os
import yaml

# =========================
# 1) 你的地图范围（50m x 50m）
# =========================
X_MIN, X_MAX = -25.0, 25.0
Y_MIN, Y_MAX = -25.0, 25.0

# =========================
# 2) 从 world 里“写死”的障碍物数据（后面你自己改这里就行）
# pose: (x, y, z, roll, pitch, yaw)   -> 平面图只用 x, y, yaw
# size: (sx, sy, sz)                  -> 平面图只用 sx, sy
# =========================
# models = [
#     {
#         "name": "obstacle1",
#         "pose": (-13.0, -7.0, 2.0, 0.0, 0.0, 0.0),
#         "size": (10.0, 6.0, 4.0),
#     },
#     {
#         "name": "obstacle2",
#         "pose": (-13.0, 7.0, 2.0, 0.0, 0.0, 0.0),
#         "size": (10.0, 6.0, 4.0),
#     },
#     {
#         "name": "obstacle3",
#         "pose": (0.0, 0.0, 2.0, 0.0, 0.0, 0.0),
#         "size": (4.0, 26.0, 4.0),
#     },
#     {
#         "name": "obstacle4",
#         "pose": (11.0, 0.0, 2.0, 0.0, 0.0, 0.0),
#         "size": (10.0, 6.0, 4.0),
#     },
# ]
models = [
    {"name": "obstacle_01", "pose": (-18.0, -18.0, 0.75, 0.0, 0.0, 0.0), "size": (1.2, 0.8, 1.5)},
    {"name": "obstacle_02", "pose": (-10.0, -20.0, 0.60, 0.0, 0.0, 0.0), "size": (0.8, 0.8, 1.2)},
    {"name": "obstacle_03", "pose": (-5.0, -8.0, 0.90, 0.0, 0.0, 0.0), "size": (1.0, 1.6, 1.8)},
    {"name": "obstacle_04", "pose": (-2.0, -14.0, 0.50, 0.0, 0.0, 0.0), "size": (1.6, 0.6, 1.0)},
    {"name": "obstacle_05", "pose": (6.0, -18.0, 0.70, 0.0, 0.0, 0.0), "size": (1.4, 1.0, 1.4)},
    {"name": "obstacle_06", "pose": (14.0, -16.0, 0.55, 0.0, 0.0, 0.0), "size": (1.0, 0.7, 1.1)},
    {"name": "obstacle_07", "pose": (20.0, -8.0, 0.80, 0.0, 0.0, 0.0), "size": (1.8, 0.9, 1.6)},
    {"name": "obstacle_08", "pose": (18.0, 2.0, 0.65, 0.0, 0.0, 0.0), "size": (1.1, 1.1, 1.3)},
    {"name": "obstacle_09", "pose": (10.0, 6.0, 0.95, 0.0, 0.0, 0.0), "size": (0.9, 1.7, 1.9)},
    {"name": "obstacle_10", "pose": (2.0, 10.0, 0.55, 0.0, 0.0, 0.0), "size": (1.5, 0.8, 1.1)},
    {"name": "obstacle_11", "pose": (-6.0, 12.0, 0.75, 0.0, 0.0, 0.0), "size": (1.2, 1.2, 1.5)},
    {"name": "obstacle_12", "pose": (-14.0, 10.0, 0.60, 0.0, 0.0, 0.0), "size": (0.8, 1.4, 1.2)},
    {"name": "obstacle_13", "pose": (-20.0, 16.0, 0.70, 0.0, 0.0, 0.0), "size": (1.3, 0.9, 1.4)},
    {"name": "obstacle_14", "pose": (-8.0, 20.0, 0.85, 0.0, 0.0, 0.0), "size": (1.7, 0.7, 1.7)},
    {"name": "obstacle_15", "pose": (6.0, 18.0, 0.65, 0.0, 0.0, 0.0), "size": (1.0, 1.5, 1.3)},
    {"name": "obstacle_16", "pose": (16.0, 16.0, 0.55, 0.0, 0.0, 0.0), "size": (1.1, 0.9, 1.1)},
    {"name": "obstacle_17", "pose": (-12.0, 0.0, 0.95, 0.0, 0.0, 0.0), "size": (0.9, 0.9, 1.9)},
    {"name": "obstacle_18", "pose": (-2.0, 2.0, 0.55, 0.0, 0.0, 0.0), "size": (1.4, 0.7, 1.1)},
    {"name": "obstacle_19", "pose": (4.0, -2.0, 0.75, 0.0, 0.0, 0.0), "size": (1.2, 1.0, 1.5)},
    {"name": "obstacle_20", "pose": (0.0, -6.0, 0.60, 0.0, 0.0, 0.0), "size": (0.8, 1.6, 1.2)},
]

# =========================
# 2.5) UWB anchors (from YAML)
# =========================
ANCHORS_YAML_PATH = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors2.yml"


def load_uwb_anchors(yaml_path: str):
    """Load anchors from UWB_Anchors.yml.

    Expected format:
      UWB_Anchors:
        - {id: 0, x: ..., y: ..., z: ...}
        - ...
    Returns:
      list of dicts with keys: id, x, y, z
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    anchors = data.get("UWB_Anchors", []) if isinstance(data, dict) else []
    # Basic robustness: keep only entries that have x/y
    out = []
    for a in anchors:
        if not isinstance(a, dict):
            continue
        if "x" not in a or "y" not in a:
            continue
        out.append({
            "id": a.get("id", -1),
            "x": float(a["x"]),
            "y": float(a["y"]),
            "z": float(a.get("z", 0.0)),
        })
    return out


def add_box(ax, center_x, center_y, sx, sy, yaw_rad, label):
    """
    在 ax 上画一个 box 障碍物（俯视图）：
      - 矩形中心：(center_x, center_y)
      - 矩形尺寸：sx (沿x方向长度), sy (沿y方向长度)
      - yaw_rad：绕中心逆时针旋转角（弧度）
    说明：
      - 你的 world 里 yaw 目前都是 0，所以矩形不会旋转。
      - 如果后面 yaw 不为 0，这里也能画“旋转矩形”。
    """
    # Rectangle 默认是用“左下角 + 宽高”定义，所以要从中心换算到左下角
    x0 = center_x - sx / 2.0
    y0 = center_y - sy / 2.0

    # 创建矩形（先画成轴对齐）
    rect = Rectangle(
        (x0, y0),
        sx,
        sy,
        linewidth=1,
        edgecolor="black",
        facecolor="black",
        fill=True
    )

    # 如果需要旋转：matplotlib 的旋转是绕 (0,0) 的，需要用变换
    if abs(yaw_rad) > 1e-9:
        # 以矩形中心为旋转中心
        t = (plt.matplotlib.transforms.Affine2D()
             .rotate_around(center_x, center_y, yaw_rad)
             + ax.transData)
        rect.set_transform(t)

    ax.add_patch(rect)


def main():
    fig, ax = plt.subplots(figsize=(7, 7))

    # =========================
    # 3) 画地图边界（50x50）
    # =========================
    border = Rectangle(
        (X_MIN, Y_MIN),
        X_MAX - X_MIN,
        Y_MAX - Y_MIN,
        linewidth=2,
        fill=False
    )
    ax.add_patch(border)

    # =========================
    # 4) 逐个画障碍物
    # =========================
    for m in models:
        x, y, z, roll, pitch, yaw = m["pose"]
        sx, sy, sz = m["size"]

        # 注意：world 的 yaw 就是绕 Z 轴的角度（弧度制）
        yaw_rad = yaw  # 你这份数据里 yaw=0

        add_box(ax, x, y, sx, sy, yaw_rad, m["name"])

    # =========================
    # 4.5) 画 UWB anchors（黄色三角）
    # =========================
    try:
        anchors = load_uwb_anchors(ANCHORS_YAML_PATH)
        ax.scatter(
            [a["x"] for a in anchors],
            [a["y"] for a in anchors],
            marker="^",
            s=60,
            c="yellow",
            edgecolors="black",
            linewidths=0.5,
            zorder=5,
        )
    except Exception as e:
        print(f"[draw_map] Failed to load anchors from {ANCHORS_YAML_PATH}: {e}")

    # =========================
    # 5) 坐标轴设置
    # =========================
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_aspect("equal", adjustable="box")

    # 不显示坐标轴、标题、网格（用于论文图）
    ax.axis("off")

    # =========================
    # 6) 保存图片到同目录下的 /paper/ 目录
    # =========================
    script_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(script_dir, "pics/paper")
    os.makedirs(save_dir, exist_ok=True)

    save_path = os.path.join(save_dir, "map_topdown.png")
    plt.savefig(save_path, dpi=300, bbox_inches="tight")

    print(f"[draw_map] figure saved to: {save_path}")

    plt.show()


if __name__ == "__main__":
    main()
