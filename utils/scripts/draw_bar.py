import numpy as np
import matplotlib.pyplot as plt

# ==============================
# 1. 在这里填写你的实验结果
# ==============================

methods = ["Minimum Snap", "Piecewise Cubic Spline", "Ours"]

# -------- cost_v --------
cost_v_values = [
    4.53,   # minimum_snap
    3.05,   # piecewise_cubic_spline
    3.32   # ours
]

# -------- cost_a --------
cost_a_values = [
    20.43,   # minimum_snap
    76.21,   # piecewise_cubic_spline
    8.51    # ours
]


# ==============================
# 2. 可调整参数
# ==============================

# 颜色（这里可以改）
colors = ["#4C72B0", "#DD8452", "#55A868"]

# 柱宽
bar_width = 0.6

# ==============================
# 3. 画图
# ==============================

fig, axes = plt.subplots(1, 2, figsize=(10, 4))

x = np.arange(len(methods))

# -------- cost_v --------
bars_v = axes[0].bar(
    x,
    cost_v_values,
    width=bar_width,
    color=colors
)

# 在柱子上显示数值
for bar in bars_v:
    height = bar.get_height()
    axes[0].text(
        bar.get_x() + bar.get_width() / 2,
        height,
        f"{height:.2f}",
        ha='center',
        va='bottom',
        fontsize=9
    )

axes[0].set_xticks(x)
axes[0].set_xticklabels(methods)

# 这里修改标题
axes[0].set_title("Velocity Cost Comparison")

# 这里修改Y轴名字
axes[0].set_ylabel("cost_v")

# -------- cost_a --------
bars_a = axes[1].bar(
    x,
    cost_a_values,
    width=bar_width,
    color=colors
)

# 在柱子上显示数值
for bar in bars_a:
    height = bar.get_height()
    axes[1].text(
        bar.get_x() + bar.get_width() / 2,
        height,
        f"{height:.2f}",
        ha='center',
        va='bottom',
        fontsize=9
    )

axes[1].set_xticks(x)
axes[1].set_xticklabels(methods)

# 这里修改标题
axes[1].set_title("Acceleration Cost Comparison")

# 这里修改Y轴名字
axes[1].set_ylabel("cost_a")

# ==============================
# 4. 版式调整
# ==============================

plt.tight_layout()

import os

# 获取当前脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# 目标保存目录：同级目录 /pics/figures/
save_dir = os.path.join(script_dir, "pics", "figures")
os.makedirs(save_dir, exist_ok=True)

save_path = os.path.join(save_dir, "trajectory_cost_comparison.png")

plt.savefig(save_path, dpi=300)
print(f"figure saved to: {save_path}")

plt.show()