import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import os
import yaml

# =========================
# 1) 你的地图范围（50m x 50m）
# =========================
X_MIN, X_MAX = -4.0, 4.0
Y_MIN, Y_MAX = -4.0, 4.0

# =========================
# 2) 从 indoor_environment2_obstacle.yml 里读取障碍物数据
# =========================
OBSTACLES_YAML_PATH = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/indoor_environment3_obstacle.yml"


def load_obstacle_models(yaml_path: str):
    """Load obstacle models from obstacle YAML.

    Expected format:
      obstacles:
        - name: obstacle_01
          cx: ...
          cy: ...
          cz: ...
          sx: ...
          sy: ...
          sz: ...

    Returns:
      list of dicts in the same format previously used by `models`:
        {
          "name": str,
          "pose": (cx, cy, cz, 0.0, 0.0, 0.0),
          "size": (sx, sy, sz),
        }
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    obstacles = data.get("obstacles", []) if isinstance(data, dict) else []
    models = []
    for obs in obstacles:
        if not isinstance(obs, dict):
            continue
        models.append({
            "name": obs.get("name", "obstacle"),
            "pose": (
                float(obs["cx"]),
                float(obs["cy"]),
                float(obs.get("cz", 0.0)),
                0.0,
                0.0,
                0.0,
            ),
            "size": (
                float(obs["sx"]),
                float(obs["sy"]),
                float(obs.get("sz", 0.0)),
            ),
        })
    return models

# =========================
# 2.5) UWB anchors (from YAML)
# =========================
ANCHORS_YAML_PATH = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors3.yml"


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
    # 4) 从 YAML 读取并逐个画障碍物
    # =========================
    try:
        models = load_obstacle_models(OBSTACLES_YAML_PATH)
    except Exception as e:
        print(f"[draw_map] Failed to load obstacles from {OBSTACLES_YAML_PATH}: {e}")
        models = []

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
